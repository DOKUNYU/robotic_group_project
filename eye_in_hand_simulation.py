import pybullet as p
import pybullet_data
import numpy as np
import cv2
import math
import yellow_detect
import pose_estimation
from time import sleep

#项目设置------------------------------------------------------------

# 连接引擎
_ = p.connect(p.GUI)
# 展示GUI的套件
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
# 添加资源路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-9.8)
time_step = 1./240. 
p.setTimeStep(time_step)

#物体生成------------------------------------------------------------

#为了生成物体，暂时关闭渲染
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

#载入外部urfd
planeUid = p.loadURDF("plane.urdf", useMaximalCoordinates=True)  # 加载一个地面
#trayUid = p.loadURDF("tray/traybox.urdf", basePosition=[0.4, 0, 0])  # 加载一个箱子，设置初始位置为（0，0，0）
pandaUid = p.loadURDF("franka_panda/panda.urdf",useFixedBase=True) #导入panda机器人

#生成不同区域
areaLength=0.5
areaWidth=0.3
areaHeight=0.001
rgbaList=[[1,1,0.4,1],[0,1,0,1],[1,0.2,0,1]]
startAreaCollisionId = p.createCollisionShape(p.GEOM_BOX,
                                  halfExtents=[areaWidth,areaLength, areaHeight])
collectAreaCollisionId = p.createCollisionShape(p.GEOM_BOX,
                                  halfExtents=[areaLength, areaWidth,areaHeight])
startAreaVisualId = p.createVisualShape(p.GEOM_BOX,
                                  halfExtents=[areaWidth, areaLength,areaHeight],
                                  rgbaColor=[0,0,0,1])
p.createMultiBody(baseMass=10,
                    baseCollisionShapeIndex=startAreaCollisionId,
                    baseVisualShapeIndex=startAreaVisualId,
                    basePosition=[0.4,0,0])
for i in range(3):
    p.createMultiBody(baseMass=10,
                    baseCollisionShapeIndex=collectAreaCollisionId,
                    baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX,
                                  halfExtents=[areaLength, areaWidth,areaHeight],
                                  rgbaColor=rgbaList[i]),
                    basePosition=[-0.7,-0.7+i*0.7,0])




#生成自定义彩色盒子尺寸
boxHalfLength = 0.0132
boxHalfWidth = 0.0132
boxHalfHeight = 0.02
#盒子的碰撞体形状
BoxCollisionId = p.createCollisionShape(p.GEOM_BOX,
                                  halfExtents=[boxHalfLength, boxHalfWidth, boxHalfHeight])
#盒子的三种颜色视觉形状
yellowBoxId = p.createVisualShape(p.GEOM_BOX,
                                  halfExtents=[boxHalfLength, boxHalfWidth, boxHalfHeight],
                                  rgbaColor=[1,1,0.4,1])
greenBoxId = p.createVisualShape(p.GEOM_BOX,
                                  halfExtents=[boxHalfLength, boxHalfWidth, boxHalfHeight],
                                  rgbaColor=[0,1,0,1])
redBoxId = p.createVisualShape(p.GEOM_BOX,
                                  halfExtents=[boxHalfLength, boxHalfWidth, boxHalfHeight],
                                  rgbaColor=[1,0.2,0,1])
pointId=p.createVisualShape(p.GEOM_BOX,
                                  halfExtents=[0.005,0.005,0.005],
                                  rgbaColor=[0,0,1,1])
#生成物体
pos1=[0.3, 0, 0.02]
pos2=[0.38, 0.08, 0.02]
pos3=[0.4, -0.16, 0.02]
p.createMultiBody(baseMass=0.5,
                    baseCollisionShapeIndex=BoxCollisionId,
                    baseVisualShapeIndex=yellowBoxId,
                    basePosition=pos1)
p.createMultiBody(baseMass=0.5,
                    baseCollisionShapeIndex=BoxCollisionId,
                    baseVisualShapeIndex=greenBoxId,
                    basePosition=pos2)
p.createMultiBody(baseMass=0.5,
                    baseCollisionShapeIndex=BoxCollisionId,
                    baseVisualShapeIndex=redBoxId,
                    basePosition=pos3)

#重新开启渲染
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

#初始设置------------------------------------------------------------

#回到初始位置，松开抓夹
rest_poses = [0,-math.pi/3,0,-math.pi*3/4,0,math.pi/2,math.pi/4]
for i in range(7):
    p.resetJointState(pandaUid,i,rest_poses[i])
p.resetJointState(pandaUid,9,0.06)
p.resetJointState(pandaUid,10,0.06)

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
pos=[-0.3,-0.55,0.5]

sleep(1)
color_detect_flag= 0 #限制颜色识别运行次数.0:未识别 1:识别完成
state_flag=0 #识别进程 0:未完成移动,1:已完成运动,未夹取,2:已夹取3:移动
finish_move_count=0 #完成移动的关节数量 一旦有未完成关节则清0
finish_grisp_time=0 #完成夹取的时间

#开始模拟

while True:
    #未完成移动
    if state_flag==0 :
        #获取joint11(gripper中心)旋转矩阵
        ee_pose_cartesian = p.getLinkState(pandaUid,11,computeForwardKinematics=1)[:1][0]
        ee_pose_quaternion = p.getLinkState(pandaUid,11,computeForwardKinematics=1)[1:2][0]
        rotation_matrix = p.getMatrixFromQuaternion(ee_pose_quaternion,physicsClientId=0)
        tx_vec = np.array([rotation_matrix[0],rotation_matrix[3],rotation_matrix[6]])
        ty_vec = np.array([rotation_matrix[1],rotation_matrix[4],rotation_matrix[7]])
        tz_vec = np.array([rotation_matrix[2],rotation_matrix[5],rotation_matrix[8]])
        # print(ee_pose_cartesian)

        base_pos = np.array(ee_pose_cartesian)
        target_pos = base_pos + 0.1*tz_vec  
        p.addUserDebugLine(base_pos,target_pos)# debug line(白线)

        #计算视角矩阵
        viewMatrix = p.computeViewMatrix(
        cameraEyePosition=base_pos,
        cameraTargetPosition=target_pos,
        cameraUpVector=tx_vec,
        physicsClientId=0)  

        # 计算投影矩阵
        projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)  
        # print(projection_matrix)
        w, h, rgb, depth, seg = p.getCameraImage(width, height, viewMatrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)

        # 转换成opencv图像
        bgr = cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)

        if color_detect_flag == 0:
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
                #print(object_camera_position)
                

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

                #标记蓝色目标点(无碰撞体)
                p.createMultiBody(baseMass=0,
                        baseVisualShapeIndex=pointId,
                        basePosition=end_effector_target_position)
                color_detect_flag += 1
            except:
                None
        try:
            # 计算逆向运动学
            joint_poses = p.calculateInverseKinematics(pandaUid,11,end_effector_target_position)
            for i in range(9):
                p.setJointMotorControl2(bodyIndex=pandaUid,
                                        jointIndex=i,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=joint_poses[i],
                                        maxVelocity=2)
            current_joint_position=p.getJointStates(pandaUid,list(range(9)))
            for i in range(9):
                if (abs(current_joint_position[i][0]-joint_poses[i]))<0.05:
                    finish_move_count+=1
                else:
                    finish_move_count=0
            if finish_move_count>=9 :
                state_flag=1
            else:
                finish_move_count=0
        except:
            None
    #已完成运动,未夹取
    elif state_flag==1 :
        p.setJointMotorControl2(pandaUid,9,p.POSITION_CONTROL,force=200)
        p.setJointMotorControl2(pandaUid,10,p.POSITION_CONTROL,force=200)
        finish_grisp_time+=time_step
        if finish_grisp_time>1 :
            state_flag=2
            finish_grisp_time=0
    #已夹取，回到初始位置
    elif state_flag==2 :
        for i in range(6):
            p.setJointMotorControl2(bodyIndex=pandaUid,
                                            jointIndex=i,
                                            controlMode=p.POSITION_CONTROL,
                                            targetPosition=rest_poses[i],
                                            maxVelocity=2)
        finish_grisp_time+=time_step
        if finish_grisp_time>1 :
            state_flag=3
            finish_grisp_time=0
            joint_poses = p.calculateInverseKinematics(pandaUid,11,pos)
    elif state_flag==3:
        for i in range(9):
            p.setJointMotorControl2(bodyIndex=pandaUid,
                                    jointIndex=i,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=joint_poses[i],
                                    maxVelocity=2)
        current_joint_position=p.getJointStates(pandaUid,list(range(9)))
        for i in range(9):
            if (abs(current_joint_position[i][0]-joint_poses[i]))<0.05:
                finish_move_count+=1
            else:
                finish_move_count=0
        if finish_move_count>=9 :
            state_flag=4
        else:
            finish_move_count=0
    elif state_flag==4 :
        p.setJointMotorControl2(pandaUid,9,p.POSITION_CONTROL,0.08)
        p.setJointMotorControl2(pandaUid,10,p.POSITION_CONTROL,0.08)
    
    cv2.imshow("bgr",bgr)
    cv2.waitKey(1)

    sleep(time_step)
    p.stepSimulation()

