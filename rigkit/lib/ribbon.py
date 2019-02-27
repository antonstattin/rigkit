"""
ribbon module contains methods to build ribbons that can be used in rigs
for twist chains and bendy parts.
"""

import math

import pymel.core as pm

from . import connect

def simple_ribbon(position_A, position_B, **kwargs):

    position_A = pm.PyNode(position_A)
    position_B = pm.PyNode(position_B)

    # query keyword arguments
    u_res = kwargs.get("uResolution", kwargs.get("ures", 1))
    v_res = kwargs.get("vResolution", kwargs.get("vres", 5))
    length_ratio = kwargs.get("lengthRatio", kwargs.get("lr", None))
    number_of_joints = kwargs.get("numberOfJoints", kwargs.get("noj", 4))
    name = kwargs.get("name", kwargs.get("n","simpleRibbon"))

    # get length ratio
    if not length_ratio:
        position_A_t = position_A.getTranslation("world")
        position_B_t = position_B.getTranslation("world")

        # calculate magnitude, length between posiition A and B
        pos_vector = [position_B_t[i] - position_A_t[i] for i in xrange(len(position_A_t))]
        length_ratio = math.sqrt(sum([pow(position_B_t[i] - position_A_t[i], 2) for i in xrange(len(position_A_t))]))


    # create nurbsplane
    nurbsplane = pm.nurbsPlane(lengthRatio=length_ratio, patchesU=u_res, patchesV=v_res, axis=[0,1,0], ch=False)[0]
    pm.setAttr(nurbsplane + ".ry", -90)
    pm.makeIdentity(nurbsplane, a=True)

    
