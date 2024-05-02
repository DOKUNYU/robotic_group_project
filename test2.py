import pybullet as p
import time
import math
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
#don't create a ground plane, to allow for gaps etc
p.resetSimulation()
p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0,0)
# 不展示GUI的套件
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
# 禁用 tinyrenderer 
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)


boxHalfLength = 1
boxHalfWidth = 0.8
boxHalfHeight = 0.7


colBoxId = p.createCollisionShape(p.GEOM_BOX,
                                  halfExtents=[boxHalfLength, boxHalfWidth, boxHalfHeight])
boxId = p.createVisualShape(p.GEOM_BOX,
                                  halfExtents=[boxHalfLength, boxHalfWidth, boxHalfHeight],
                                  rgbaColor=[0,0,0,1])

mass = 1
visualShapeId = -1


for i in range(5):
  p.createMultiBody(baseMass=0.5,
                    baseCollisionShapeIndex=colBoxId,
                    baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX,
                                  halfExtents=[boxHalfLength, boxHalfWidth, boxHalfHeight],
                                  rgbaColor=[0+i*0.2,0+i*0.2,0+i*0.2,1]),
                    basePosition=[0, -4+i*2, 3])

p.setGravity(0,0,-9.8)
time_step = 1./240.
p.setTimeStep(time_step)


p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
while (1):
  p.stepSimulation()
  #print(keys)
  time.sleep(0.01)