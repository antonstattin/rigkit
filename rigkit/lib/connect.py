
import pymel.core as pm

from . import util, matrix
reload(matrix)

def attach_to_nurbsurface(driven, nurbssurface, **kwargs):
    """ Creates a rivet on a nurbs surface



        :param kwargs:
            name - n : name of setup
            maintainOffset - mo: maintain offset
            scale - s: connect scale
            rotate - ro: connect rotate
            translate - t: connect translate
   """

    nurbssurface = util.get_pynodes(nurbssurface)[0]
    driven = util.get_pynodes(driven)[0]

    # check if we have a shape and not the transform
    if pm.objectType(nurbssurface) != "nurbsSurface":
        shape = pm.listRelatives(nurbssurface, c=True, type="shape")
        if not shape: raise ValueError('%s have no shape'%nurbssurface)
        nurbssurface = shape[0]


    name = kwargs.get("n", kwargs.get("name", "surfaceAttached"))
    closest_point = kwargs.get("cp", kwargs.get("closestPoint", True))
    u_value = kwargs.get("u", kwargs.get("uValue", 0))
    v_value = kwargs.get("v", kwargs.get("vValue", 0))
    maintain_offset = kwargs.get("mo", kwargs.get("maintainOffset", False))

    # create nodes
    fbf_matrix = pm.createNode("fourByFourMatrix", n="_%s_FBFM"%name)
    decompose_matrix = pm.createNode("decomposeMatrix", n="_%s_DEMAT"%name)
    posi = pm.createNode("pointOnSurfaceInfo", n="_%s_POSI"%name)


    if closest_point:
        cpos_tmp = pm.createNode("closestPointOnSurface", n="_%s_TMP_CPOS"%name)
        nurbssurface.worldSpace[0] >> cpos_tmp.inputSurface

        position = pm.xform(driven, t=True, ws=True, q=True)
        cpos_tmp.inPosition.inPositionX.set(position[0])
        cpos_tmp.inPosition.inPositionY.set(position[1])
        cpos_tmp.inPosition.inPositionZ.set(position[2])

        u_value = cpos_tmp.result.parameterU.get()
        v_value = cpos_tmp.result.parameterV.get()

        pm.delete(cpos_tmp)


    # connect point on surface info
    nurbssurface.worldSpace[0] >> posi.inputSurface

    posi.parameterU.set(u_value)
    posi.parameterV.set(v_value)

    # connect four by four matrix
    posi.result.normal.normalX >> fbf_matrix.in00
    posi.result.normal.normalY >> fbf_matrix.in01
    posi.result.normal.normalZ >> fbf_matrix.in02

    posi.result.tangentU.tangentUx >> fbf_matrix.in10
    posi.result.tangentU.tangentUy >> fbf_matrix.in11
    posi.result.tangentU.tangentUz >> fbf_matrix.in12

    posi.result.tangentV.tangentVx >> fbf_matrix.in20
    posi.result.tangentV.tangentVy >> fbf_matrix.in21
    posi.result.tangentV.tangentVz >> fbf_matrix.in22

    posi.result.position.positionX >> fbf_matrix.in30
    posi.result.position.positionY >> fbf_matrix.in31
    posi.result.position.positionZ >> fbf_matrix.in32

    fbf_matrix.output >> decompose_matrix.inputMatrix

    if maintain_offset:
        mult_matrix = pm.createNode("multMatrix", n="_%s_MMAT"%name)

        # lazy.. create a transform to extract offset matrix
        tmp_transform = pm.group(n="_tmp_transform", em=True)
        decompose_matrix.outputTranslate >> tmp_transform.translate
        decompose_matrix.outputRotate >> tmp_transform.rotate
        offset_matrix = matrix.get_local_offset(tmp_transform, driven)
        offset_matrix = [offset_matrix(i, j) for i in range(4) for j in range(4)]
        pm.setAttr(mult_matrix + ".matrixIn[0]", offset_matrix, type="matrix")

        fbf_matrix.output >> mult_matrix.matrixIn[1]

        pm.connectAttr(mult_matrix + ".matrixSum",
        decompose_matrix + ".inputMatrix", f=True)

        # disconnect and delete tmp loc
        decompose_matrix.outputTranslate // tmp_transform.translate
        decompose_matrix.outputRotate // tmp_transform.rotate
        pm.delete(tmp_transform)

    decompose_matrix.outputTranslate >> driven.translate
    decompose_matrix.outputRotate >> driven.rotate


def parent(*arg, **kwargs):
    """ Creates a parent constraint with matrix nodes. If multiple drivers are
        passed into this function, a weight blend setup will be created.

        NOTE: Blended rotations seems not to work like mayas parent constraint.

        :param *arg: drivers ending with the driven transform
        :type *arg: transforms

        :param kwargs:
             name - n : name of nodes
             maintainOffset - mo : add offset matrix
             preventBenginCycle - pbc: skip cycle, connect parent inv matrix directly
             scale - s: connect scale
             rotate - ro: connect rotate
             translate - t: connect translate
    """


    # query keyword arguments
    name = kwargs.get('n', kwargs.get('name', 'parentMatrix'))
    maintain_offset = kwargs.get("mo", kwargs.get("maintainOffset", True))
    prevent_benign_cycle = kwargs.get("pbc", kwargs.get("preventBenginCycle", True))
    do_scale = kwargs.get("s", kwargs.get("scale", False))
    do_rotate = kwargs.get("ro", kwargs.get("rotate", True))
    do_translate = kwargs.get("t", kwargs.get("translate", True))

    if not 2 <= len(arg):
        raise ValueError('need driver and driven object as first argument')


    # query drivers + driven
    drivers = util.get_pynodes(arg[:-1])
    driven = util.get_pynodes(arg[-1:])[0]


    # create matrix nodes
    mult_matrix = pm.createNode("multMatrix", n="_%s_MMAT"%name)
    decompose_matrix = pm.createNode("decomposeMatrix", n="_%s_DEMAT"%name)
    wt_matrix = None

    # increment, used for keeping track of inputs
    input_count = 0


    if len(drivers) > 1:
        drivers_outputs = [driver.worldMatrix[0] for driver in drivers]

        if maintain_offset:
            mult_matrix_sums = []
            for i, output in enumerate(drivers_outputs):

                mult_m_name = "_{name}_offset_{i}_MMAT"
                mult_matrix_offset = pm.createNode("multMatrix",
                                            n=mult_m_name.format(name=name, i=i+1))

                # set offset matrix
                offset_matrix = matrix.get_local_offset(drivers[i], driven)
                offset_matrix = [offset_matrix(i, j) for i in range(4) for j in range(4)]

                pm.setAttr(mult_matrix_offset + ".matrixIn[0]", offset_matrix, type="matrix")

                # connect driver worldMatrix
                output >> mult_matrix_offset.matrixIn[1]

                # override list
                mult_matrix_sums.append(mult_matrix_offset.matrixSum)

            # override driver .worldMatrix outputs with multMatrix matrixSum output
            drivers_outputs = mult_matrix_sums

        # create weighted matrix
        wt_matrix = matrix.weight_matrix(*tuple(drivers_outputs + [mult_matrix.matrixIn[0]]), n=name)
        input_count = 1

    else:
        driver_output = drivers[0].worldMatrix[0]

        if maintain_offset:
            offset_matrix = matrix.get_local_offset(drivers[0], driven)
            offset_matrix = [offset_matrix(i, j) for i in range(4) for j in range(4)]


            pm.setAttr(mult_matrix + ".matrixIn[0]", offset_matrix)

            driver_output >> mult_matrix.matrixIn[1]
            input_count = 2

        else:
            driver_output >> mult_matrix.matrixIn[0]
            input_count = 1



    if prevent_benign_cycle:
        # get the parent if none then use world and skip
        parent = pm.listRelatives(driven, p=True)

        if parent:
            driven_input = "{mmult}.matrixIn[{count}]".format(mmult=mult_matrix,
                                                              count=input_count)
            pm.connectAttr(parent[0] + ".worldInverseMatrix[0]", driven_input)
            input_count += 1
    else:
        driven_input = "{mmult}.matrixIn[{count}]".format(mmult=mult_matrix,
                                                          count=input_count)
        pm.connectAttr(driven[0] + ".parentInverseMatrix[0]", driven_input)
        input_count += 1


    mult_matrix.matrixSum >> decompose_matrix.inputMatrix

    if do_translate: decompose_matrix.outputTranslate >> driven.translate
    if do_rotate: decompose_matrix.outputRotate >> driven.rotate
    if do_scale: decompose_matrix.outputScale >> driven.scale

    return decompose_matrix, wt_matrix

def point(*arg, **kwargs):
    """ Create a point constraint, connecting only translation channels """

    # query keyword arguments
    name = kwargs.get('n', kwargs.get('name', 'pointMatrix'))
    maintain_offset = kwargs.get("mo", kwargs.get("maintainOffset", True))
    prevent_benign_cycle = kwargs.get("pbc", kwargs.get("preventBenginCycle", True))

    return parent(*arg, n=name, mo=maintain_offset, pbc=prevent_benign_cycle, s=False, ro=False, t=True)

def orient(*arg, **kwargs):
    """ Create a orient constraint, connecting only rotation channels """

    # query keyword arguments
    name = kwargs.get('n', kwargs.get('name', 'orientMatrix'))
    maintain_offset = kwargs.get("mo", kwargs.get("maintainOffset", True))
    prevent_benign_cycle = kwargs.get("pbc", kwargs.get("preventBenginCycle", True))

    return parent(*arg, n=name, mo=maintain_offset, pbc=prevent_benign_cycle, s=False, ro=True, t=False)


def scale(*arg, **kwargs):
    """ Create a scale constraint, connecting only scaling channels """

    # query keyword arguments
    name = kwargs.get('n', kwargs.get('name', 'scaleMatrix'))
    maintain_offset = kwargs.get("mo", kwargs.get("maintainOffset", True))
    prevent_benign_cycle = kwargs.get("pbc", kwargs.get("preventBenginCycle", True))

    return parent(*arg, n=name, mo=maintain_offset, pbc=prevent_benign_cycle, s=True, ro=False, t=False)
