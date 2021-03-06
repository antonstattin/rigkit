""" matrix methods to 'keep code dry' """


import pymel.core as pm
import maya.OpenMaya as om

from . import util

def get_local_offset(parent, child):

    parentWorldMatrix = util.get_dag_path(parent).inclusiveMatrix()
    childWorldMatrix = util.get_dag_path(child).inclusiveMatrix()

    return childWorldMatrix * parentWorldMatrix.inverse()


def prevent_benign_cycle(matrix_mult, matrix_in_index, driven):
    """ Connects parent if any else we expect it to be parented to world
        and do nothing
    """

    # get the parent if none then use world and skip
    parent = pm.listRelatives(driven, p=True)

    if parent:
        driven_input = "{mmult}.matrixIn[{count}]".format(mmult=matrix_mult,
                                                          count=matrix_in_index)
        pm.connectAttr(parent[0] + ".worldInverseMatrix[0]", driven_input)
        return True

    else: return False



def weight_matrix(*arg, **kwarg):
    """ Creates a weighted matrix
        :arg: drivers, driven (type: str or pymel.core.general.Attribute)
        :kwarg: n - name  = name of wtAddMatrix

        :return: wtAddMatrix node
    """

    name = kwarg.get("n", kwarg.get("name", "weightMatrix"))

    wt_matrix = pm.createNode("wtAddMatrix", n="_%s_WTMAT"%name)

    drivers_output = arg[:-1]
    driven_input = arg[-1:][0]

    print drivers_output, arg

    weight = 1.0/len(drivers_output)

    print weight
    for i, output in enumerate(drivers_output):
        matrixIn = "{wtMat}.wtMatrix[{i}].matrixIn"
        weightIn = "{wtMat}.wtMatrix[{i}].weightIn"

        pm.connectAttr(output, matrixIn.format(wtMat=wt_matrix, i=i))
        pm.setAttr(weightIn.format(wtMat=wt_matrix, i=i), weight)

    # connect attribute
    pm.connectAttr(wt_matrix + ".matrixSum", driven_input)

    return wt_matrix
