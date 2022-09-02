from cgitb import grey
from sys import prefix
import cv2 as cv
import cv2.aruco as aruco
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from matplotlib import pyplot as plt
import numpy as np
import glob
import argparse

bridge = CvBridge()


class sub_class:
    def __init__(self):
        self.image_sub = rospy.Subscriber('/zed2/zed_node/rgb_raw/image_raw_color', Image, self.image_sub_cb)
        self.ros_image = Image()
    def image_sub_cb(self, data):
        self.ros_image = data

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def calibrate(dirpath, prefix, image_format, square_size, width=9, height=6):
    """ Apply camera calibration operation for images in the given directory path. """
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
    objp = np.zeros((height*width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

    objp = objp * square_size

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    if dirpath[-1:] == '/':
        dirpath = dirpath[:-1]

    images = glob.glob(dirpath+'/' + prefix + '*.' + image_format)

    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (width, height), None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)

            corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv.drawChessboardCorners(img, (width, height), corners2, ret)

    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    return [ret, mtx, dist, rvecs, tvecs]


def save_coefficients(mtx, dist, path):
    """ Save the camera matrix and the distortion coefficients to given path/file. """
    cv_file = cv.FileStorage(path, cv.FILE_STORAGE_WRITE)
    cv_file.write("K", mtx)
    cv_file.write("D", dist)
    # note you *release* you don't close() a FileStorage object
    cv_file.release()




if __name__ == "__main__":
    rospy.init_node("opencv_example")

    # subs = sub_class()
    # rospy.sleep(2)
    # image_message = subs.ros_image
    # desired_encoding = image_message.encoding

    # cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding=desired_encoding)
    
    # gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)


    
    # Display the resulting frame
    # cv.imshow('frame', gray)
    # Save the images
    # status = cv.imwrite('/home/liu/franka_rl/src/opencv_examples/scripts/calibration_samples/img_20.png', gray)

    
    parser = argparse.ArgumentParser(description='Camera calibration')
    parser.add_argument('--image_dir', type=str, required=True, help='image directory path')
    parser.add_argument('--image_format', type=str, required=True,  help='image format, png/jpg')
    parser.add_argument('--prefix', type=str, required=True, help='image prefix')
    parser.add_argument('--square_size', type=float, required=False, help='chessboard square size')
    parser.add_argument('--width', type=int, required=False, help='chessboard width size, default is 9')
    parser.add_argument('--height', type=int, required=False, help='chessboard height size, default is 6')
    parser.add_argument('--save_file', type=str, required=True, help='YML file to save calibration matrices')


    image_dir = '/home/liu/franka_rl/src/opencv_examples/scripts/calibration_samples'
    prefix = 'img_'
    image_format = 'png'
    square_size = 1.5
    width = 9
    height = 6
    ret, mtx, dist, rvecs, tvecs = calibrate(image_dir, prefix, image_format, square_size, width, height)


    save_coefficients(mtx, dist, image_dir + '/camera.yml')







    
    # plt.imshow(gray, 'gray')

    # plt.show()