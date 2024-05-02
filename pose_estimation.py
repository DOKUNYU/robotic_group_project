import numpy as np
import cv2

# 以选取点为原点，画出x,y,z三轴的投影
def draw(img, corners, imgpts):
    corner = list(corners[0].ravel())
    corner[0] = int(corner[0])
    corner[1] = int(corner[1])
    pts_store = [0,0,0]
    for i in range(0,3):
        pts_store[i] = [int(list(imgpts[i].ravel())[0]),int(list(imgpts[i].ravel())[1])]

    img = cv2.line(img, corner, pts_store[0], (255, 0, 0), 5)
    img = cv2.line(img, corner, pts_store[1], (0, 255, 0), 5)
    img = cv2.line(img, corner, pts_store[2], (0, 0, 255), 5)

def calculate_intrin_matrix(width,height,projection_matrix):
    fx = projection_matrix[0]*width/2.
    fy = projection_matrix[5]*height/2.
    cx = width/2.
    cy = height/2.
    intr_matrix = np.array([[fx, 0,  cx],  # 内参
                            [0,  fy, cy], 
                            [0,  0,  1]],dtype = np.double)
    return intr_matrix

def calculate_pose(imgpoints,objpoints,intrin_matrix,img):
    width = 32
    height = 32
    dist_coeffs = np.zeros((5,1),dtype = np.double) # 畸变系数
    ret,rotation_vector,translation_vector = cv2.solvePnP(objpoints,imgpoints,intrin_matrix,dist_coeffs,flags=2)
    rotation_matrix = cv2.Rodrigues(rotation_vector)[0]
    axis = np.float32([[100-width/2,-height/2,0], [-width/2,100-height/2,0], [-width/2,-height/2,100]]).reshape(-1,3)
    proj_imgpts, jac = cv2.projectPoints(axis, rotation_vector, translation_vector, intrin_matrix, dist_coeffs)
    draw(img,imgpoints,proj_imgpts)
    return rotation_matrix,translation_vector