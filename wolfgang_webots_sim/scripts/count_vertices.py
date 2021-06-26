# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from controller import Supervisor
import trimesh
import traceback
import transforms3d
import math
import numpy as np
import time
import os

ROBOT_NAME = "RED_PLAYER_1"
JOINT_TYPES = ["HingeJoint", "HingeJointWithBacklash", "Hinge2Joint", "Hinge2JointWithBacklash"]

MODEL_NAME = None
ROBOT_PATH = None
ROBOT_DIR = None
EXPORT_MF = True


def spawn_robot():
    """Spawn and returns the robot that should be verified"""
    string = f'DEF {ROBOT_NAME} {MODEL_NAME}' \
        '{name "red player 1" translation 0 0 0 rotation 0 0 1 0 controller "void"}'
    s.getRoot().getField('children').importMFNodeFromString(-1, string)
    return s.getFromDef(ROBOT_NAME)

def despawn_robot():
    s.getRoot().getField('children').removeMF(-1, )

def get_node_desc(node):
    # TODO ideally the node path should be provided but it's unclear if that can still be accessed from the dictionary
    # represenation
    node_type = node[1].get("__type")
    node_name = node[1].get("name", "unknown")
    return f"(name: {node_name}, type: {node_type})"


def build_dict_field(field):
    """
    :type field: Field
    """
    if field is None:
        return None

    type_name = field.getTypeName()
    value_s = None
    if type_name == "SFBool":
        value_s = field.getSFBool()
    elif type_name == "SFInt32":
        value_s = field.getSFInt32()
    elif type_name == "SFFloat":
        value_s = field.getSFFloat()
    elif type_name == "SFVec2f":
        value_s = field.getSFVec2f()
    elif type_name == "SFVec3f":
        value_s = field.getSFVec3f()
    elif type_name == "SFRotation":
        value_s = field.getSFRotation()
    elif type_name == "SFColor":
        value_s = field.getSFColor()
    elif type_name == "SFString":
        value_s = field.getSFString()
    elif type_name == "SFNode":
        value_s = build_dict_node(field.getSFNode())
    elif type_name == "MFNode":
        vals = []
        for i in range(field.getCount()):
            vals.append(build_dict_node(field.getMFNode(i)))
        value_s = vals
    elif EXPORT_MF:
        if type_name == "MFBool":
            vals = []
            for i in range(field.getCount()):
                vals.append(field.getMFBool(i))
            value_s = vals
        elif type_name == "MFInt32":
            vals = []
            for i in range(field.getCount()):
                vals.append(field.getMFInt32(i))
            value_s = vals
        elif type_name == "MFFloat":
            vals = []
            for i in range(field.getCount()):
                vals.append(field.getMFFloat(i))
            value_s = vals
        elif type_name == "MFVec2f":
            vals = []
            for i in range(field.getCount()):
                vals.append(field.getMFVec2f(i))
            value_s = vals
        elif type_name == "MFVec3f":
            vals = []
            for i in range(field.getCount()):
                vals.append(field.getMFVec3f(i))
            value_s = vals
        elif type_name == "MFColor":
            vals = []
            for i in range(field.getCount()):
                vals.append(field.getMFColor(i))
            value_s = vals
        elif type_name == "MFRotation":
            vals = []
            for i in range(field.getCount()):
                vals.append(field.getMFRotation(i))
            value_s = vals
        elif type_name == "MFString":
            vals = []
            for i in range(field.getCount()):
                vals.append(field.getMFString(i))
            value_s = vals
        else:
            print(f"type {type_name} not known")

    return type_name, value_s

def build_dict_node(node):
    """
    :param node: Node
    :return:
    """
    if node is None:
        return {}
    local_fields = {}
    local_fields["__type"] = node.getTypeName()
    nb_fields = node.getProtoNumberOfFields()
    for i in range(nb_fields):
        field = node.getProtoFieldByIndex(i)
        if field is None:
            print(f"None field reached {i+1}/{nb_fields} in {get_node_desc(('SFNode',local_fields))}\n")
            continue
        local_fields[field.getName()] = build_dict_field(field)
    return local_fields


def mesh_to_indexed_face_set(mesh):
    coordIndex = []
    for face in mesh.faces:
        coordIndex.extend(face)
        coordIndex.append(-1)
    coordinates = []
    for vert in mesh.vertices:
        coordinates.append(vert)
    return "SFNode", \
           {"__type": "IndexedFaceSet",
            "coord": ("SFNode", {"__type": "Coordinate", "point": ("MFVec3f", coordinates)}),
            "coordIndex": ("MFInt32", coordIndex)}

def get_meshes(node):
    """Return a list of tuples (mesh, joint_and_tf, ??)"""
    coords = []
    if node[0] == "SFNode":
        # already a coordinate
        if node[1].get("__type") == "Coordinate" is not None:
            coord = node[1]["point"][1]
            coords.append(coord)
        # go to next solids through all fields that are nodes
        for k, v in node[1].items():
            coords.extend(get_meshes(v))
    elif node[0] == "MFNode":
        for child_node in node[1]:
            coords.extend(get_meshes(("SFNode", child_node)))
    return coords


s = Supervisor()
try:
    for required_variable in ["ROBOT_NAME", "ROBOT_PATH"]:
        if required_variable not in os.environ:
            raise RuntimeError(f"Environment variable {required_variable} is missing")
    if "EXPORT_MF" in os.environ:
        EXPORT_MF = bool(os.environ["EXPORT_MF"])
        print(f"export mf : {EXPORT_MF}")
    MODEL_NAME = os.environ["ROBOT_NAME"]
    ROBOT_PATH = os.environ["ROBOT_PATH"]
    ROBOT_DIR = os.path.dirname(ROBOT_PATH)

    robot_node = spawn_robot()
    robot = build_dict_node(robot_node)
    meshes = get_meshes(("SFNode", robot))
    num_vert = np.sum(meshes)
    print(f"There are {num_vert} in the IndexFaceSets of the {MODEL_NAME}")
    despawn_robot()
except Exception:
    print(f"Failed execution of model verifier with error:\n{traceback.format_exc()}\n")

