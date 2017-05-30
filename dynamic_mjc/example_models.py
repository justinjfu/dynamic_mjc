from .model_builder import MJCModel
import numpy as np


def block_push(object_pos=(0,0,0), goal_pos=(0,0,0)):
    mjcmodel = MJCModel('block_push')
    mjcmodel.root.compiler(inertiafromgeom="true",angle="radian",coordinate="local")
    mjcmodel.root.option(timestep="0.01",gravity="0 0 0",iterations="20",integrator="Euler")
    default = mjcmodel.root.default()
    default.joint(armature='0.04', damping=1, limited='true')
    default.geom(friction=".8 .1 .1",density="300",margin="0.002",condim="1",contype="1",conaffinity="1")

    worldbody = mjcmodel.root.worldbody()

    palm = worldbody.body(name='palm', pos=[0,0,0])
    palm.geom(type='capsule', fromto=[0,-0.1,0,0,0.1,0], size=.12)
    proximal1 = palm.body(name='proximal_1', pos=[0,0,0])
    proximal1.joint(name='proximal_j_1', type='hinge', pos=[0,0,0], axis=[0,1,0], range=[-2.5,2.3])
    proximal1.geom(type='capsule', fromto=[0,0,0,0.4,0,0], size=0.06, contype=1, conaffinity=1)
    distal1 = proximal1.body(name='distal_1', pos=[0.4,0,0])
    distal1.joint(name = "distal_j_1", type = "hinge", pos = "0 0 0", axis = "0 1 0", range = "-2.3213 2.3", damping = "1.0")
    distal1.geom(type="capsule", fromto="0 0 0 0.4 0 0", size="0.06", contype="1", conaffinity="1")
    distal2 = distal1.body(name='distal_2', pos=[0.4,0,0])
    distal2.joint(name="distal_j_2",type="hinge",pos="0 0 0",axis="0 1 0",range="-2.3213 2.3",damping="1.0")
    distal2.geom(type="capsule",fromto="0 0 0 0.4 0 0",size="0.06",contype="1",conaffinity="1")
    distal4 = distal2.body(name='distal_4', pos=[0.4,0,0])
    distal4.site(name="tip arml",pos="0.1 0 -0.2",size="0.01")
    distal4.site(name="tip armr",pos="0.1 0 0.2",size="0.01")
    distal4.joint(name="distal_j_3",type="hinge",pos="0 0 0",axis="1 0 0",range="-3.3213 3.3",damping="0.5")
    distal4.geom(type="capsule",fromto="0 0 -0.2 0 0 0.2",size="0.04",contype="1",conaffinity="1")
    distal4.geom(type="capsule",fromto="0 0 -0.2 0.2 0 -0.2",size="0.04",contype="1",conaffinity="1")
    distal4.geom(type="capsule",fromto="0 0 0.2 0.2 0 0.2",size="0.04",contype="1",conaffinity="1")

    object = worldbody.body(name='object', pos=object_pos)
    object.geom(rgba="1. 1. 1. 1",type="box",size="0.05 0.05 0.05",density='0.00001',contype="1",conaffinity="1")
    object.joint(name="obj_slidez",type="slide",pos="0.025 0.025 0.025",axis="0 0 1",range="-10.3213 10.3",damping="0.5")
    object.joint(name="obj_slidex",type="slide",pos="0.025 0.025 0.025",axis="1 0 0",range="-10.3213 10.3",damping="0.5")
    distal10 = object.body(name='distal_10', pos=[0,0,0])
    distal10.site(name='obj_pos', pos=[0.025,0.025,0.025], size=0.01)

    goal = worldbody.body(name='goal', pos=goal_pos)
    goal.geom(rgba="1. 0. 0. 1",type="box",size="0.1 0.1 0.1",density='0.00001',contype="0",conaffinity="0")
    distal11 = goal.body(name='distal_11', pos=[0,0,0])
    distal11.site(name='goal_pos', pos=[0.05,0.05,0.05], size=0.01)


    actuator = mjcmodel.root.actuator()
    actuator.motor(joint="proximal_j_1",ctrlrange="-2 2",ctrllimited="true")
    actuator.motor(joint="distal_j_1",ctrlrange="-2 2",ctrllimited="true")
    actuator.motor(joint="distal_j_2",ctrlrange="-2 2",ctrllimited="true")
    actuator.motor(joint="distal_j_3",ctrlrange="-2 2",ctrllimited="true")

    return mjcmodel


EAST = 0
WEST = 1
NORTH = 2
SOUTH = 3

def twod_corridor(direction=EAST, length=1.2):
    mjcmodel = MJCModel('twod_corridor')
    mjcmodel.root.compiler(inertiafromgeom="true", angle="radian", coordinate="local")
    mjcmodel.root.option(timestep="0.01", gravity="0 0 0", iterations="20", integrator="Euler")
    default = mjcmodel.root.default()
    default.joint(damping=1, limited='false')
    default.geom(friction=".5 .1 .1", density="1000", margin="0.002", condim="1", contype="2", conaffinity="1")

    worldbody = mjcmodel.root.worldbody()

    particle = worldbody.body(name='particle', pos=[0,0,0])
    particle.geom(name='particle_geom', type='sphere', size='0.03', rgba='0.0 0.0 1.0 1', contype=1)
    particle.site(name='particle_site', pos=[0,0,0], size=0.01)
    particle.joint(name='ball_x', type='slide', pos=[0,0,0], axis=[1,0,0])
    particle.joint(name='ball_y', type='slide', pos=[0,0,0], axis=[0,1,0])

    pos = np.array([0.0,0,0])
    if direction == EAST or direction == WEST:
        pos[0] = length-0.1
    else:
        pos[1] = length-0.1
    if direction == WEST or direction == SOUTH:
        pos = -pos

    target = worldbody.body(name='target', pos=pos)
    target.geom(name='target_geom', conaffinity=2, type='sphere', size=0.02, rgba=[0,0.9,0.1,1])

    # arena
    if direction == EAST:
        L = -0.1
        R = length
        U = 0.1
        D = -0.1
    elif direction == WEST:
        L = -length
        R = 0.1
        U = 0.1
        D = -0.1
    elif direction == SOUTH:
        L = -0.1
        R = 0.1
        U = 0.1
        D = -length
    elif direction == NORTH:
        L = -0.1
        R = 0.1
        U = length
        D = -0.1

    worldbody.geom(conaffinity=1, fromto=[L, D, .01, R, D, .01], name="sideS", rgba="0.9 0.4 0.6 1", size=.02, type="capsule")
    worldbody.geom(conaffinity=1, fromto=[R, D, .01, R, U, .01], name="sideE", rgba="0.9 0.4 0.6 1", size=".02", type="capsule")
    worldbody.geom(conaffinity=1, fromto=[L, U, .01, R, U, .01], name="sideN", rgba="0.9 0.4 0.6 1", size=".02", type="capsule")
    worldbody.geom(conaffinity=1, fromto=[L, D, .01, L, U, .01], name="sideW", rgba="0.9 0.4 0.6 1", size=".02", type="capsule")

    actuator = mjcmodel.root.actuator()
    actuator.motor(joint="ball_x", ctrlrange=[-1.0, 1.0], ctrllimited=True)
    actuator.motor(joint="ball_y", ctrlrange=[-1.0, 1.0], ctrllimited=True)

    return mjcmodel
