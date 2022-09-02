from argparse import HelpFormatter
from configparser import Interpolation
import time
import cv2 as cv
import os
import numpy as np
import matplotlib.pyplot as plt
import pyrealsense2 as rs

from gtts import gTTS


os.chdir(os.path.dirname(__file__))



def bounding_boxes(depth_img, color_img):
    # b,g,r = cv.split(img)

    depth_mask = np.uint8(depth_img[:,:,2] > 200) * 255

    observe_img = cv.resize(depth_mask, (100, 100))

    cv.imwrite('/home/liu/panda_tamp/src/pddlstream/examples/pybullet/panda/rs_obs.png', observe_img)

    kernel = np.ones((5,5), np.uint8)
    dilated_depth_mask = cv.dilate(depth_mask, kernel=kernel, iterations=1)
    cv.imshow('dilated_depth_mask',dilated_depth_mask)
    # cubes = np.uint8(mask) * 255

    # cv.imshow('mask',mask)


    # Outline in color_img
    gray = cv.cvtColor(color_img, cv.COLOR_RGB2GRAY)
    # Apply Gaussian blur with a X-Y Sigma of 50:
    # gray = cv.GaussianBlur(gray, (3,3), 50, 50 );
    # cv.imshow('gray',gray)
    
    # Get edge
    canny = cv.Canny(gray, 50, 20)
    


    lines = cv.morphologyEx(canny, cv.MORPH_DILATE, (18,18))
    big_lines = cv.dilate(lines, kernel=kernel, iterations=2)
    cv.imshow('big_lines',big_lines)


    mask = np.uint8((dilated_depth_mask[:,:] > 200) & (big_lines[:,:] < 200)) * 255
    cv.imshow('mask',mask)






    # # Noise removal
    # kernel = np.ones((3,3),np.uint8)
    # opening = cv.morphologyEx(mask,cv.MORPH_OPEN,kernel, iterations = 2)
    # # cv.imshow('opening',opening)

    # # Sure background area
    # sure_bg = cv.dilate(opening,kernel,iterations=3)
    # # cv.imshow('sure_bg',sure_bg)

    # # Finding sure foreground area
    # dist_transform = cv.distanceTransform(opening,cv.DIST_L2,5)
    # ret, sure_fg = cv.threshold(dist_transform,0.7*dist_transform.max(),255,0)
    # # cv.imshow('sure_fg',sure_fg)

    # # Finding unknown region
    # sure_fg = np.uint8(sure_fg)
    # unknown = cv.subtract(sure_bg,sure_fg)
    # # cv.imshow('unknown',unknown)


    #contours, hierarchy = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)

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

def get_positions():


    
    try:
        # Create a context object. This object owns the handles to all connected realsense devices
        pipeline = rs.pipeline()

        # Configure streams
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30) # 640, 480
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Align
        align_to = rs.stream.color
        align = rs.align(align_to)



        # Start streaming
        profile = pipeline.start(config)

        # get camera intrinsics
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()



        x_buffer = []
        y_buffer = []
        start_time = time.time()

        while True:
            dic_list = []
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
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

            img, rects = bounding_boxes(depth_colormap, color_image)

            # print(boxes[0])
            bias = [0.03, 0.02]
            if len(rects) >0:
                for rect in rects:
                    
                    cv.circle(img,(np.int0(rect[0])[0], np.int0(rect[0])[1]), 4, (0,255,0), thickness=4)
                    dist = depth_frame.get_distance(np.int0(rect[0])[0], np.int0(rect[0])[1])

                    #calculate real world coordinates
                    Ytemp = dist*(rect[0][0] -intr.ppx)/intr.fx
                    Xtemp = dist*(rect[0][1] -intr.ppy)/intr.fy
                    Ztemp = dist

                    rotation = rect[-1]

                    x_buffer.append(Xtemp)
                    y_buffer.append(Ytemp)

                    dic_list.append((rects.index(rect), [Xtemp + 0.5 + bias[0], Ytemp + bias[1], rotation]))

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
    if len(obj_dict) == 0:
        mytext = 'There is no block on the table, what do you want from me?'
    elif len(obj_dict) == 1:
        mytext = 'There is just' + str(len(obj_dict)) +'blocks on the table! Cannot you just do it by yourself?'
    elif len(obj_dict) >= 6 and len(obj_dict) <= 8:
        mytext = 'There are' + str(len(obj_dict)) +'blocks on the table! This is too much work for me.'
    elif len(obj_dict) > 8:
        mytext = 'There are..' + 'Too many fucking blocks on this table.'
    else:
        mytext = 'There are' + str(len(obj_dict)) +'blocks on the table! Do not touch them cause I will not observe again!'
    language = 'en'
    myobj = gTTS(text=mytext, lang=language, slow=False)
    myobj.save("welcome.mp3")
    os.system("mpg321 welcome.mp3")



    return obj_dict

if __name__ == "__main__":
    # os.system('cd ~/panda_tamp')
    # os.system('. ~/panda_tamp/devel/setup.bash')
    ans = get_positions()
    print(ans) 
    print(len(ans))