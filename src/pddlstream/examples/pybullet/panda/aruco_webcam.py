from cgitb import grey
from pickletools import float8
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

def init_list_of_objects(size):
    list_of_objects = list()
    for i in range(0,size):
        list_of_objects.append( list() ) #different object reference each time
    return list_of_objects

# Get positions
def get_positions(n_blocks = 3):




    # Parameter prepare
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters_create()
    image_dir = '/home/liu/panda_pddlstream/src/pddlstream/examples/pybullet/panda/calibration_samples'
    matrixes = load_coefficients(image_dir + '/camera_webcam.yml')
    matrix_coefficients = matrixes[0]
    distortion_coefficients = matrixes[1]


    x_buffer = init_list_of_objects(n_blocks)
    y_buffer = init_list_of_objects(n_blocks)
    yaw_buffer = init_list_of_objects(n_blocks)


    positions = np.empty((n_blocks, 3), dtype=np.float64)
    orientations = np.empty((n_blocks, 3), dtype=np.float64)

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


        if np.all(ids is not None):
            # print(ids.shape[0])
            if ids.shape[0] - 1 >= n_blocks: # -1 for the reference mark
            
                # Find reference
                if [0] in ids: #np.where(ids == [0]).shape[0]
                    rvec_0, tvec_0, markerPoints_0 = aruco.estimatePoseSingleMarkers(corners[np.where(ids == [0])[0][0]], 0.04, matrix_coefficients, distortion_coefficients)
                    aruco.drawAxis(cv_image, matrix_coefficients, distortion_coefficients, rvec_0, tvec_0, 0.1)
                


                    position_ref = [0.7, -0.2, 0.05]
                    # print(ids)
                    
                    corners = list(corners)
                    del corners[np.where(ids == [0])[0][0]]
                    ids = np.delete(ids, np.where(ids == [0])[0][0])

                    
                    ids_sorted = np.sort(ids)
                    
                    
                    
                    
                    # Here we only consider the aruco 0 and 1
                    for i in range(n_blocks):
                        
                        rvec, tvec, markerPoints_0 = aruco.estimatePoseSingleMarkers(corners[i], 0.04, matrix_coefficients, distortion_coefficients)
                        aruco.drawAxis(cv_image, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.1)
                        composed_rvec, composed_tvec = relativePosition(rvec, tvec, rvec_0, tvec_0)
                        
                        block_x = position_ref[0] + composed_tvec[0][0]
                        block_y = position_ref[1] + composed_tvec[1][0]
                        block_z = position_ref[2] + composed_tvec[2][0]

                        # print(ids)

                        sorted_index = np.where(ids_sorted == ids[i])[0][0]
                        # print(i)
                        # print(ids[i])
                        # print(sorted_index)

                        x_buffer[sorted_index].append(block_x) # The index of buffer starts from 1, while the ids starts from 0 (includes reference)
                        y_buffer[sorted_index].append(block_y)
                        yaw_buffer[sorted_index].append(composed_rvec[2][0])

                        # print(x_buffer)
                        # print('one inner loop')
                    
                    if len(x_buffer[0]) >= 100:
                        break

     

        cv.imshow('frame', cv_image)

        # Stop the code
        key = cv.waitKey(3) & 0xFF
        if key == ord('q'):  # Quit
            break

   

    for j in range(n_blocks):
        positions[j][0] = sum(x_buffer[j])/len(x_buffer[j])
        positions[j][1] = sum(y_buffer[j])/len(y_buffer[j])
        orientations[j][2] = sum(yaw_buffer[j])/len(yaw_buffer[j])


    print(f'orientations = {orientations}')
    print(f'positions = {positions}')
    return positions, orientations

if __name__ == "__main__":
    # rospy.init_node('opencv_example')
    get_positions()

