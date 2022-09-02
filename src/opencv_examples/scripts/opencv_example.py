from cgitb import grey
import cv2 as cv
import cv2.aruco as aruco
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from matplotlib import pyplot as plt
import numpy as np
import time

bridge = CvBridge()


class sub_class:
    def __init__(self):
        self.image_sub = rospy.Subscriber('/zed2/zed_node/rgb_raw/image_raw_color', Image, self.image_sub_cb)
        self.ros_image = Image()
    def image_sub_cb(self, data):
        self.ros_image = data


# Get calibration matrixes
def load_coefficients(path):
    """ Loads camera matrix and distortion coefficients. """
    # FILE_STORAGE_READ
    cv_file = cv.FileStorage(path, cv.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("K").mat()
    dist_matrix = cv_file.getNode("D").mat()

    cv_file.release()
    return [camera_matrix, dist_matrix]


# Get inverse vectors
def inversePerspective(rvec, tvec):
    R, _ = cv.Rodrigues(rvec)
    R = np.matrix(R).T
    invTvec = np.dot(R, np.matrix(-tvec))
    invRvec, _ = cv.Rodrigues(R)
    return invRvec, invTvec

# Get relative position
def relativePosition(rvec1, tvec1, rvec2, tvec2):
    """ Get relative position for rvec2 & tvec2. Compose the returned rvec & tvec to use composeRT with rvec2 & tvec2 """
    rvec1, tvec1 = rvec1.reshape((3, 1)), tvec1.reshape((3, 1))
    rvec2, tvec2 = rvec2.reshape((3, 1)), tvec2.reshape((3, 1))
    # Inverse the second marker
    invRvec, invTvec = inversePerspective(rvec2, tvec2)
    info = cv.composeRT(rvec1, tvec1, invRvec, invTvec)
    composedRvec, composedTvec = info[0], info[1]
    composedRvec = composedRvec.reshape((3, 1))
    composedTvec = composedTvec.reshape((3, 1))
    return composedRvec, composedTvec

if __name__ == "__main__":

     
    rospy.init_node("opencv_example")
    subs = sub_class()
    rospy.sleep(2)


    # Parameter prepare
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()
    image_dir = '/home/liu/franka_rl/src/opencv_examples/scripts/calibration_samples'
    matrixes = load_coefficients(image_dir + '/camera.yml')
    matrix_coefficients = matrixes[0]
    distortion_coefficients = matrixes[1]

    # Start loop
    while True:
        # Get an image from ROS
        image_message = subs.ros_image
        desired_encoding = image_message.encoding
        cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding=desired_encoding)
        gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        # cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2BGR)

        # Detect ArUco mark
        corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict,
                                                                    parameters=parameters,
                                                                    cameraMatrix=matrix_coefficients,
                                                                    distCoeff=distortion_coefficients)

        


        # print(ids)
        # print(np.where(ids == [1])[0].shape)
        # print(corners[np.where(ids == [1])[0][0]])
 
        # Draw the detection
        if np.all(ids is not None):  # If there are markers found by detector
            for i in range(0, len(ids)):  # Iterate in markers
                # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.04, matrix_coefficients, distortion_coefficients)
                (rvec - tvec).any()  # get rid of that nasty numpy value array error
                # Draw Axis
                # aruco.drawDetectedMarkers(cv_image, corners)  # Draw A square around the markers
                # aruco.drawAxis(cv_image, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)


        # Here we only consider the aruco 0 and 1
            if [0] in ids: #np.where(ids == [0]).shape[0]
                rvec_0, tvec_0, markerPoints_0 = aruco.estimatePoseSingleMarkers(corners[np.where(ids == [0])[0][0]], 0.04, matrix_coefficients, distortion_coefficients)
                aruco.drawAxis(cv_image, matrix_coefficients, distortion_coefficients, rvec_0, tvec_0, 0.1)
            if [1] in ids: #np.where(ids == [0]).shape[0]
                rvec_1, tvec_1, markerPoints_0 = aruco.estimatePoseSingleMarkers(corners[np.where(ids == [1])[0][0]], 0.04, matrix_coefficients, distortion_coefficients)
                aruco.drawAxis(cv_image, matrix_coefficients, distortion_coefficients, rvec_1, tvec_1, 0.1)

            if [0] in ids and [1] in ids:
                composed_rvec, composed_tvec = relativePosition(rvec_0, tvec_0, rvec_1, tvec_1)

                position_ref = [0.7, -0.2, 0.05]
                block_x = position_ref[0] - composed_tvec[0][0]
                block_y = position_ref[1] - composed_tvec[1][0]
                block_z = position_ref[2] - composed_tvec[2][0]

                print([block_x, block_y, block_z])


                
                
            
                # print(composed_tvec)

        

        cv.imshow('frame', cv_image)

        # Stop the code
        key = cv.waitKey(3) & 0xFF
        if key == ord('q'):  # Quit
            break


