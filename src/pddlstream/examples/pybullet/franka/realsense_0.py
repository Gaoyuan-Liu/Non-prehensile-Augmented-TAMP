from argparse import HelpFormatter
from configparser import Interpolation
import time
import cv2 as cv
import os
import numpy as np
import matplotlib.pyplot as plt
import pyrealsense2 as rs

from gtts import gTTS
from numpy import zeros, newaxis


os.chdir(os.path.dirname(__file__))

CAM_POSITION_OBJ = [0.4, 0.0, 0.65]
CAM_POSITION_PLATE = [0.0, 0.4, 0.65]

class Camera():

    def __init__(self):
        # Create a context object. This object owns the handles to all connected realsense devices
        self.pipeline = rs.pipeline()

        # Configure streams
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30) # 640, 480
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Align
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        # Start streaming
        profile = self.pipeline.start(config)

        # get camera intrinsics
        self.intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()


    def bounding_boxes(self, depth_img, color_img):

        depth_mask = np.uint8(depth_img[:,:,2] > 220) * 255

        kernel = np.ones((5,5), np.uint8)
        dilated_depth_mask = cv.dilate(depth_mask, kernel=kernel, iterations=1)
        
        # Outline in color_img
        gray = cv.cvtColor(color_img, cv.COLOR_RGB2HSV)[:,:,2]
        cv.imshow('gray', gray)


        
        # Get edge
        canny = cv.Canny(gray, 50, 20) # 50, 20
        


        lines = cv.morphologyEx(canny, cv.MORPH_DILATE, (18,18))
        big_lines = cv.dilate(lines, kernel=kernel, iterations=1)
        cv.imshow('big_lines',big_lines)


        mask = np.uint8((dilated_depth_mask[:,:] > 200) & (big_lines[:,:] < 200)) * 255
        cv.imshow('mask',mask)

        contours, hierarchy = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

        contours_filtered = []



        for i in range(len(contours)):
            if cv.contourArea(contours[i]) >=250:
                contours_filtered.append(contours[i])


        cv.drawContours(depth_img, contours_filtered, -1, (255,0,0), 2)

        final = depth_img
        
        boxes = []
        
        if len(contours_filtered) >= 1:

            for i in contours_filtered:

                rect = cv.minAreaRect(i)
                box = cv.boxPoints(rect)
                box = np.int0(box)
                boxes.append(rect)
                final = cv.drawContours(depth_img,[box],0,(0,255,0),2)

        return final, boxes

    def bounding_boxes_plate(self, depth_img, color_img):


        depth_mask = np.uint8((depth_img > 160) & (depth_img < 170)) * 255 #& (depth_img[:,:,2] < 210)) * 255
        kernel = np.ones((5,5), np.uint8)
        dilated_depth_mask = cv.dilate(depth_mask, kernel=kernel, iterations=1)


        cv.imshow('depth_mask', depth_mask)

        

        mask = np.uint8((dilated_depth_mask > 200)) * 255
        # cv.imshow('mask',mask)

        contours, hierarchy = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

        contours_filtered = []



        for i in range(len(contours)):
            if cv.contourArea(contours[i]) >=17000:
                contours_filtered.append(contours[i])


        cv.drawContours(depth_img, contours_filtered, -1, (255,0,0), 2)

        final = depth_img
        
        boxes = []
        
        if len(contours_filtered) >= 1:

            for i in contours_filtered:

                rect = cv.minAreaRect(i)
                box = cv.boxPoints(rect)
                box = np.int0(box)
                boxes.append(rect)
                final = cv.drawContours(depth_img,[box],0,(0,255,0),2)

        return final, boxes

    def get_positions(self):
        try:
            x_buffer = []
            y_buffer = []
            start_time = time.time()

            while True:
                dic_list = []
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                # This call waits until a new coherent set of frames is available on a device
                # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
                
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                if not depth_frame: continue
                


                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # img, boxes = bounding_boxes(color_image)
                depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.4), cv.COLORMAP_JET) # 0.4

                # plt.imshow(depth_colormap)
                # plt.show()

                img, rects = self.bounding_boxes(depth_colormap, color_image)
                
                

                # print(boxes[0])
                bias = [0.04, 0.02]
                if len(rects) >0:
                    for rect in rects:
                        
                        cv.circle(img,(np.int0(rect[0])[0], np.int0(rect[0])[1]), 4, (0,255,0), thickness=4)
                        dist = depth_frame.get_distance(np.int0(rect[0])[0], np.int0(rect[0])[1])

                        #calculate real world coordinates
                        Ytemp = dist*(rect[0][0] -self.intr.ppx)/self.intr.fx
                        Xtemp = dist*(rect[0][1] -self.intr.ppy)/self.intr.fy
                        Ztemp = dist

                        rotation = rect[-1]
                        #print(rotation)

                        x_buffer.append(Xtemp)
                        y_buffer.append(Ytemp)



                        dic_list.append((rects.index(rect), [Xtemp + CAM_POSITION_OBJ[0] + bias[0], Ytemp + CAM_POSITION_OBJ[1] +bias[1], rotation]))

                    if len(x_buffer) >= 300:
                        break
                
                now_time = time.time()
                dtime = now_time-start_time
                if dtime >= 5:
                    break

                cv.imshow('depth', img)

                cv.waitKey(1)
                
            
        except Exception as e:
            print(e)
            pass

        obj_dict = dict(dic_list)


        # Voice report
        # if len(obj_dict) == 0:
        #     mytext = 'There is no block on the table, what do you want from me?'
        # elif len(obj_dict) == 1:
        #     mytext = 'There is just' + str(len(obj_dict)) +'blocks on the table! Cannot you just do it by yourself?'
        # elif len(obj_dict) >= 6 and len(obj_dict) <= 8:
        #     mytext = 'There are' + str(len(obj_dict)) +'blocks on the table! This is too much work for me, you will pay for this.'
        # elif len(obj_dict) > 8:
        #     mytext = 'There are..' + 'Too many fucking blocks on this table.'
        # else:
        #     mytext = 'There are' + str(len(obj_dict)) +'blocks on the table! Do not touch them cause I will not observe again!'
        # language = 'en'
        # myobj = gTTS(text=mytext, lang=language, slow=False)
        # myobj.save("welcome.mp3")
        # os.system("mpg321 welcome.mp3")

        return obj_dict

    def get_plate_position(self):
        try:
            x_buffer = []
            y_buffer = []
            start_time = time.time()

            while True:
                dic_list = []
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                # This call waits until a new coherent set of frames is available on a device
                # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
                
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                if not depth_frame: continue
                


                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # img, boxes = bounding_boxes(color_image)
                # depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.4), cv.COLORMAP_JET) # 0.4

                depth_colormap = cv.convertScaleAbs(depth_image, alpha=0.3)

                # plt.imshow(depth_colormap)
                # plt.show()

                img, rects = self.bounding_boxes_plate(depth_colormap, color_image)
                
                

                # print(boxes[0])
                bias = [0.04, 0.02]
                if len(rects) > 0:
                    for rect in rects:
                        
                        cv.circle(img,(np.int0(rect[0])[0], np.int0(rect[0])[1]), 4, (0,255,0), thickness=4)
                        dist = depth_frame.get_distance(np.int0(rect[0])[0], np.int0(rect[0])[1])

                        #calculate real world coordinates
                        Ytemp = dist*(rect[0][0] -self.intr.ppx)/self.intr.fx
                        Xtemp = dist*(rect[0][1] -self.intr.ppy)/self.intr.fy
                        Ztemp = dist

                        rotation = rect[-1]
                        #print(rotation)

                        x_buffer.append(Xtemp)
                        y_buffer.append(Ytemp)



                        dic_list.append((rects.index(rect), [Xtemp + CAM_POSITION_PLATE[0] + bias[0], Ytemp + CAM_POSITION_PLATE[1] + bias[1], rotation]))

                    if len(x_buffer) >= 100:
                        break
                
                now_time = time.time()
                dtime = now_time-start_time
                if dtime >= 5:
                    break

                cv.imshow('depth', img)

                cv.waitKey(1)
                
            
        except Exception as e:
            print(e)
            pass

        obj_dict = dict(dic_list)
        self.pipeline.stop()

        return obj_dict


    def observe(self):
        try:
            
            start_time = time.time()

            while True:
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                if not depth_frame: continue
                
                color_image = np.asanyarray(color_frame.get_data())
                # cv.imshow('color_image', color_image)

                depth_image = np.asanyarray(depth_frame.get_data())
                # cv.imshow('depth_image', depth_image*100)
                color_image = np.asanyarray(color_frame.get_data())


                csa = cv.convertScaleAbs(depth_image, alpha=0.4)
                # cv.imshow('csa', csa)

                # depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, 0.4), cv.COLORMAP_JET) # 0.4
                # cv.imshow('depth_colormap', depth_colormap)
                depth_mask = np.uint8((csa > 200) & (csa < 230)) * 255
                depth_mask[:40, :] = 0
                # cv.imshow('depth_mask', depth_mask)
                observe_img = cv.resize(depth_mask, (100, 100))
                cv.imshow('observe_img', observe_img)
                
                # cv.rotate(observe_img, cv.ROTATE_180)
                
                now_time = time.time()
                dtime = now_time-start_time
                if dtime >= 5:
                    break


                cv.waitKey(1)
                
            
        except Exception as e:
            print(e)
            pass
        # cv.imwrite('/home/liu/panda_tamp/src/pddlstream/examples/pybullet/franka/rs_obs2.png', observe_img)
        return observe_img[newaxis,:,:]


if __name__ == "__main__":
    # os.system('cd ~/panda_tamp')
    # os.system('. ~/panda_tamp/devel/setup.bash')
    camera = Camera()
    ans = camera.get_positions()
