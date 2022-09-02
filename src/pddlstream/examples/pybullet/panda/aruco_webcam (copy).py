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
        self.image_sub = rospy.Subscriber('/zed2/zed_node/rgb/image_rect_color', Image, self.image_sub_cb)
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

# Get positions
def get_positions(n = 1):




    # Parameter prepare
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters_create()
    image_dir = '/home/liu/panda_pddlstream/src/pddlstream/examples/pybullet/panda/calibration_samples'
    matrixes = load_coefficients(image_dir + '/camera_webcam.yml')
    matrix_coefficients = matrixes[0]
    distortion_coefficients = matrixes[1]

    print(matrix_coefficients)



    x_buffer1 = []
    y_buffer1 = []
    x_buffer2 = []
    y_buffer2 = []
    x_buffer3 = []
    y_buffer3 = []

    cap = cv.VideoCapture(4)
    # Start loop
    while True:
        ret, cv_image = cap.read()
        
        gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        # cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2BGR)

        # Detect ArUco mark
        corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict,
                                                                    parameters=parameters,
                                                                    cameraMatrix=matrix_coefficients,
                                                                    distCoeff=distortion_coefficients)

        

 
        # Draw the detection
        if np.all(ids is not None):  # If there are markers found by detector


            # Here we only consider the aruco 0 and 1
            if [0] in ids: #np.where(ids == [0]).shape[0]
                rvec_0, tvec_0, markerPoints_0 = aruco.estimatePoseSingleMarkers(corners[np.where(ids == [0])[0][0]], 0.04, matrix_coefficients, distortion_coefficients)
                aruco.drawAxis(cv_image, matrix_coefficients, distortion_coefficients, rvec_0, tvec_0, 0.1)
            if [1] in ids: #np.where(ids == [0]).shape[0]
                rvec_1, tvec_1, markerPoints_0 = aruco.estimatePoseSingleMarkers(corners[np.where(ids == [1])[0][0]], 0.04, matrix_coefficients, distortion_coefficients)
                aruco.drawAxis(cv_image, matrix_coefficients, distortion_coefficients, rvec_1, tvec_1, 0.1)
            if [2] in ids: #np.where(ids == [0]).shape[0]
                rvec_2, tvec_2, markerPoints_0 = aruco.estimatePoseSingleMarkers(corners[np.where(ids == [2])[0][0]], 0.04, matrix_coefficients, distortion_coefficients)
                aruco.drawAxis(cv_image, matrix_coefficients, distortion_coefficients, rvec_2, tvec_2, 0.1)
            if [3] in ids: #np.where(ids == [0]).shape[0]
                rvec_3, tvec_3, markerPoints_0 = aruco.estimatePoseSingleMarkers(corners[np.where(ids == [3])[0][0]], 0.04, matrix_coefficients, distortion_coefficients)
                aruco.drawAxis(cv_image, matrix_coefficients, distortion_coefficients, rvec_3, tvec_3, 0.1)

            # if [6] in ids: #np.where(ids == [0]).shape[0]
            #     rvec_3, tvec_3, markerPoints_0 = aruco.estimatePoseSingleMarkers(corners[np.where(ids == [6])[0][0]], 0.04, matrix_coefficients, distortion_coefficients)
            #     aruco.drawAxis(cv_image, matrix_coefficients, distortion_coefficients, rvec_3, tvec_3, 0.1)

            if [0] in ids and [1] in ids and [2] in ids and [3] in ids:
                composed_rvec_01, composed_tvec_01 = relativePosition(rvec_1, tvec_1, rvec_0, tvec_0)
                composed_rvec_02, composed_tvec_02 = relativePosition(rvec_2, tvec_2, rvec_0, tvec_0)
                composed_rvec_03, composed_tvec_03 = relativePosition(rvec_3, tvec_3, rvec_0, tvec_0)

                position_ref = [0.7, -0.2, 0.05]
                block1_x = position_ref[0] + composed_tvec_01[0][0]
                block1_y = position_ref[1] + composed_tvec_01[1][0]
                block1_z = position_ref[2] + composed_tvec_01[2][0]
                block2_x = position_ref[0] + composed_tvec_02[0][0]
                block2_y = position_ref[1] + composed_tvec_02[1][0]
                block2_z = position_ref[2] + composed_tvec_02[2][0]
                block3_x = position_ref[0] + composed_tvec_03[0][0]
                block3_y = position_ref[1] + composed_tvec_03[1][0]
                block3_z = position_ref[2] + composed_tvec_03[2][0]

                # print([block_x, block_y, block_z])
                # print(f'composed_tvec = {composed_tvec}')
                # print(f'composed_rvec = {composed_rvec}')



                
                x_buffer1.append(block1_x)
                y_buffer1.append(block1_y)
                x_buffer2.append(block2_x)
                y_buffer2.append(block2_y)
                x_buffer3.append(block3_x)
                y_buffer3.append(block3_y)

            if len(x_buffer1) >= 100 and len(y_buffer2) >= 100:
                break



        

        cv.imshow('frame', cv_image)

        # Stop the code
        key = cv.waitKey(3) & 0xFF
        if key == ord('q'):  # Quit
            break

    block1_x = sum(x_buffer1)/len(x_buffer1)
    block1_y = sum(y_buffer1)/len(y_buffer1)
    block2_x = sum(x_buffer2)/len(x_buffer2)
    block2_y = sum(y_buffer2)/len(y_buffer2)
    block3_x = sum(x_buffer3)/len(x_buffer3)
    block3_y = sum(y_buffer3)/len(y_buffer3)
    

    positions = [[block1_x, block1_y, block1_z], [block2_x, block2_y, block2_z], [block3_x, block3_y, block3_z]]
    orientations = [[composed_rvec_01[0][0], composed_rvec_01[1][0], composed_rvec_01[2][0]], 
                    [composed_rvec_02[0][0], composed_rvec_02[1][0], composed_rvec_02[2][0]],
                    [composed_rvec_03[0][0], composed_rvec_03[1][0], composed_rvec_03[2][0]]]

    print(f'orientations = {orientations}')
    return positions, orientations

if __name__ == "__main__":
    # rospy.init_node('opencv_example')
    get_positions()

