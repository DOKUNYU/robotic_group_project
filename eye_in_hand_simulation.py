import pybullet as p
import pybullet_data
import numpy as np
import cv2
import math
import yellow_detect
import pose_estimation
from time import sleep

# 连接引擎
_ = p.connect(p.GUI)
# 不展示GUI的套件
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
# 添加资源路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())


planeUid = p.loadURDF("plane.urdf", useMaximalCoordinates=True)  # 加载一个地面
trayUid = p.loadURDF("tray/traybox.urdf", basePosition=[0.4, 0, 0])  # 加载一个箱子，设置初始位置为（0，0，0）
pandaUid = p.loadURDF("franka_panda/panda.urdf",useFixedBase=True)
sphereUid = p.loadURDF("sphere_small.urdf",np.array( [0.6, 0.2, 0.1]))
lego_Uid = p.loadURDF("lego/lego.urdf",np.array([0.3, -0.10, 0.01]))

rest_poses = [0,-math.pi/3,0,-math.pi*3/4,0,math.pi/2,math.pi/4]
for i in range(7):
    p.resetJointState(pandaUid,i,rest_poses[i])

p.setGravity(0,0,-9.8)
time_step = 1./240.
p.setTimeStep(time_step)

width = 1080  # 图像宽度
height = 720   # 图像高度

fov = 50  # 相机视角
aspect = width / height  # 宽高比
near = 0.01  # 最近拍摄距离
far = 20  # 最远拍摄距离

# 物体的世界坐标
width_box = 32
height_box = 32
obj_left_up = (-width_box/2,-height_box/2,0)
obj_right_up = (width_box/2,-height_box/2,0)
obj_right_down = (width_box/2,height_box/2,0)
obj_left_down = (-width_box/2,height_box/2,0)
objpoints = np.array([obj_left_up,obj_right_up,obj_right_down,obj_left_down],dtype=np.double)

sleep(1)
count = 0

while True:

    ee_pose_cartesian = p.getLinkState(pandaUid,11,computeForwardKinematics=1)[:1][0]
    ee_pose_quaternion = p.getLinkState(pandaUid,11,computeForwardKinematics=1)[1:2][0]
    rotation_matrix = p.getMatrixFromQuaternion(ee_pose_quaternion,physicsClientId=0)
    tx_vec = np.array([rotation_matrix[0],rotation_matrix[3],rotation_matrix[6]])
    ty_vec = np.array([rotation_matrix[1],rotation_matrix[4],rotation_matrix[7]])
    tz_vec = np.array([rotation_matrix[2],rotation_matrix[5],rotation_matrix[8]])
    # print(tx_vec)

    base_pos = np.array(ee_pose_cartesian)
    target_pos = base_pos + 0.1*tz_vec
    p.addUserDebugLine(base_pos,target_pos)


    viewMatrix = p.computeViewMatrix(
    cameraEyePosition=base_pos,
    cameraTargetPosition=target_pos,
    cameraUpVector=tx_vec,
    physicsClientId=0)  # 计算视角矩阵

    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)  # 计算投影矩阵
    # print(projection_matrix)
    w, h, rgb, depth, seg = p.getCameraImage(width, height, viewMatrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)

    # 转换成opencv图像
    bgr = cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)

    if count == 0:
        try:
            # 检测黄色方块
            pixel_left_up,pixel_right_up,pixel_right_down,pixel_left_down = yellow_detect.yellow_detect(bgr)
            imgpoints = np.array([pixel_left_up,pixel_right_up,pixel_right_down,pixel_left_down], dtype = np.double)

            # 计算相机内参
            intrin_matrix = pose_estimation.calculate_intrin_matrix(width,height,projection_matrix)

            # 计算位姿,得到目标物体在相机坐标系下的齐次矩阵
            camera_rotation_matrix,translation_vector = pose_estimation.calculate_pose(imgpoints,objpoints,intrin_matrix,bgr)
            object_camera_position = np.array(translation_vector).T[0]
            object_camera_position = [x/1000. for x in object_camera_position]
            # print(object_camera_position)
            

            object_homo_matrix = np.array([[camera_rotation_matrix[0][0],camera_rotation_matrix[0][1],camera_rotation_matrix[0][2],object_camera_position[0]],
                                        [camera_rotation_matrix[1][0],camera_rotation_matrix[1][1],camera_rotation_matrix[1][2],object_camera_position[1]],
                                        [camera_rotation_matrix[2][0],camera_rotation_matrix[2][1],camera_rotation_matrix[2][2],object_camera_position[2]],
                                        [0                           ,0                           ,0                           ,1]])
            
            # 相机的位姿不同于末端的位姿，需要绕z轴旋转90度
            transformation_matrix = np.array([[0,-1,0],
                                              [1,0,0],
                                              [0,0,1]])
            # rotation_matrix = transformation_matrix @ np.array(rotation_matrix).reshape(3,3)
            rotation_matrix = np.array(rotation_matrix).reshape(3,3) @ transformation_matrix

            # 计算相机在基坐标系下的齐次矩阵
            camera_homo_matrix = np.array([[rotation_matrix[0][0],rotation_matrix[0][1],rotation_matrix[0][2],base_pos[0]],
                                           [rotation_matrix[1][0],rotation_matrix[1][1],rotation_matrix[1][2],base_pos[1]],
                                           [rotation_matrix[2][0],rotation_matrix[2][1],rotation_matrix[2][2],base_pos[2]],
                                           [0                    ,0                    ,0                    ,1]])

            # 坐标变换,得到物体在基坐标系下的位姿
            base_homo_matrix = camera_homo_matrix @ object_homo_matrix

            # 机械臂末端到达的目标点
            end_effector_target_position = [base_homo_matrix[0][3],base_homo_matrix[1][3],base_homo_matrix[2][3]]
            print(end_effector_target_position)


        except:
            None

    count = count + 1

    # 计算逆向运动学
    try:
        # print(end_effector_target_position)
        # print(object_camera_position)
        joint_poses = p.calculateInverseKinematics(pandaUid,11,end_effector_target_position)

        
        for i in range(9):
            p.setJointMotorControl2(bodyIndex=pandaUid,
                                    jointIndex=i,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=joint_poses[i],
                                    maxVelocity=math.pi/4)
    except:
        None


    
    cv2.imshow("bgr",bgr)
    cv2.waitKey(1)

    sleep(time_step)
    p.stepSimulation()

