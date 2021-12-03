

import argparse
import struct
import sys
import copy

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface


def load_gazebo_models(table_pose, white_squares, black_squares, white_pieces, black_pieces, block_reference_frame, table_reference_frame):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples' ) +"/models/"
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xm l =table_file.read().replace('\n', '')
    # Load White Block URDF
    white_block_xml = ''
    with open (model_path + "block_w/model.urdf", "r") as block_file:
        white_block_xm l =block_file.read().replace('\n', '')

    # Load Black Block URDF
    black_block_xml = ''
    with open (model_path + "block_b/model.urdf", "r") as block_file:
        black_block_xm l =block_file.read().replace('\n', '')

    # Load White Square URDF
    white_square_xml = ''
    with open(model_path + "white_square/model.urdf", "r") as block_file:
        white_square_xml = block_file.read().replace('\n', '')

    # Load Black Square URDF
    black_square_xml = ''
    with open(model_path + "black_square/model.urdf", "r") as block_file:
        black_square_xml = block_file.read().replace('\n', '')

    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    # SPAWNING White Squares
    for i in range(len(white_squares)):
        square = white_squares[i]
        name = "w_square_" + str(i)
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        try:
            spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            resp_urdf = spawn_urdf(name, white_square_xml, "/",
                                   square, block_reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    # SPAWNING Black Squares
    for i in range(len(black_squares)):
        square = black_squares[i]
        name = "b_square_" + str(i)
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        try:
            spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            resp_urdf = spawn_urdf(name, black_square_xml, "/",
                                   square, block_reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    # SPAWNING White Blocks
    for i in range(len(white_pieces)):
        square = white_pieces[i]
        name = "w_piece_" + str(i)
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        try:
            spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            resp_urdf = spawn_urdf(name, white_block_xml, "/",
                                   square, block_reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    # SPAWNING Black Blocks
    for i in range(len(black_pieces)):
        square = black_pieces[i]
        name = "b_piece_" + str(i)
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        try:
            spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            resp_urdf = spawn_urdf(name, black_block_xml, "/",
                                   square, block_reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn URDF service call failed: {0}".format(e))



def delete_gazebo_models(white_squares, black_squares, white_pieces, black_pieces):
    """
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    for i in range(len(white_squares)):
        rospy.wait_for_service('/gazebo/delete_model')
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        try:
            resp_delete = delete_model("w_square_" + str(i))
        except rospy.ServiceException, e:
            rospy.loginfo("Delete Model service call failed: {0}".format(e))
    for i in range(len(black_squares)):
        rospy.wait_for_service('/gazebo/delete_model')
        try:
            resp_delete = delete_model("b_square_" + str(i))
        except rospy.ServiceException, e:
            rospy.loginfo("Delete Model service call failed: {0}".format(e))
    for i in range(len(white_pieces)):
        rospy.wait_for_service('/gazebo/delete_model')
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        try:
            resp_delete = delete_model("w_block_" + str(i))
        except rospy.ServiceException, e:
            rospy.loginfo("Delete Model service call failed: {0}".format(e))
    for i in range(len(black_pieces)):
        rospy.wait_for_service('/gazebo/delete_model')
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        try:
            resp_delete = delete_model("b_block_" + str(i))
        except rospy.ServiceException, e:
            rospy.loginfo("Delete Model service call failed: {0}".format(e))

    rospy.wait_for_service('/gazebo/delete_model')
    try:
        resp_delete = delete_model("cafe_table" + str(i))
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))
    """

