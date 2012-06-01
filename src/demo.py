from pandac.PandaModules import Light,AmbientLight,DirectionalLight
from pandac.PandaModules import NodePath
from pandac.PandaModules import Vec3,Vec4,Mat4,VBase4,Point3
from direct.task.Task import Task

from tree2 import *

from direct.showbase.ShowBase import ShowBase
base = ShowBase()
base.disableMouse()
#base.cam.setPos(0, -80, 10)
t = DefaultTree()
t.reparentTo(base.render)
#make an optimized snapshot of the current tree
np = t.getStatic()
np.setPos(10, 10, 0)
np.reparentTo(base.render)
#demonstrate growing
last = [0] # a bit hacky
def grow(task):
    if task.time > last[0] + 1:
        t.grow()
        last[0] = task.time
        #t.leaves.detachNode()
    if last[0] > 10:
        return task.done
    return task.cont
base.taskMgr.add(grow, "growTask")


dlight = DirectionalLight('dlight')

dlnp = render.attachNewNode(dlight)
dlnp.setHpr(0, 0, 0)
render.setLight(dlnp)

alight = AmbientLight('alight')

alnp = render.attachNewNode(alight)
render.setLight(alnp)

#rotating light to show that normals are calculated correctly
def updateLight(task):
    base.camera.setHpr(task.time/20.0*360,0,0)
   
   
    base.camera.setPos(0,0,0)
    base.camera.setPos(base.cam,0,-80,15)
    base.camera.setP(-4)
   
    h=task.time/10.0*360+180
   
    dlnp.setHpr(0,h,0)
    h=h+90
    h=h%360
    h=min(h,360-h)
    #h is now angle from straight up
    hv=h/180.0
    hv=1-hv
    sunset=max(0,1.0-abs(hv-.5)*8)
    if hv>.5: sunset=1
    #sunset=sunset**.2
    sunset=VBase4(0.8, 0.5, 0.0, 1)*sunset
    sun=max(0,hv-.5)*2*4
    sun=min(sun,1)
    dlight.setColor((VBase4(0.4, 0.9, 0.8, 1)*sun*2+sunset))
    alight.setColor(VBase4(0.2, 0.2, 0.3, 1)*sun+VBase4(0.2, 0.2, 0.3, 1)+sunset*.2)
    return Task.cont   

taskMgr.add(updateLight, "rotating Light")

base.run()