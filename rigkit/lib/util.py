"""
This module contains methods to help with smaller repiting tasks from
getting the dag path and checking for PyNodes etc.
"""

import pymel.core as pm
import maya.OpenMaya as om


# --------------------------
#    OpenMaya Utils
# --------------------------

def get_dag_path(node=None):
    """ return MDagPath of node """
    m_sel = om.MSelectionList()
    m_sel.add(node)
    m_dagpath = om.MDagPath()
    m_sel.getDagPath(0, m_dagpath)

    return m_dagpath



# --------------------------
#    Pymel Utils
# --------------------------

def get_pynodes(nodes):
    """ returns a list of pynodes from nodes """

    if not isinstance(nodes, list) and not isinstance(nodes, tuple):
        nodes = [nodes]

    return [pm.PyNode(node) for node in nodes]
