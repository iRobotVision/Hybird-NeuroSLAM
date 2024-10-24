#!/usr/bin/env python
import rospy
import numpy as np
from ratslam_ros.msg import gaussianCells
from ClassShowHDCell import ClassShowHDCell
from ClassShowGridCell import ClassShowGridCell
import sys
import matplotlib

curr_coordinate = ''
cali_coordinate = ''
inject_coordinate = ''
est_coordinate = ''

update_gridInfo_flag = False


def gaussianCells_callback(data):
    global curr_coordinate
    global cali_coordinate
    global inject_coordinate
    global est_coordinate
    global update_gridInfo_flag

    curr_coordinate = np.array(data.msg_curr_Coordinate, dtype=np.float32)
    cali_coordinate = np.array(data.msg_cali_Coordinate, dtype=np.float32)
    inject_coordinate = np.array(data.msg_inject_Coordinate, dtype=np.float32)
    est_coordinate = np.array(data.msg_est_Coordinate, dtype=np.float32)

    update_gridInfo_flag = True


# main function
if __name__ == '__main__':
    rospy.init_node('ShowCells', anonymous=True)
    rospy.Subscriber('/gaussianCells', gaussianCells, gaussianCells_callback)
    str_node_type = str(sys.argv[0])
    str_node_type = str_node_type[str_node_type.rfind('/') + 1:]
    rospy.loginfo("ShowCells:%s", str_node_type)

    shHD = ClassShowHDCell()
    shPosition = ClassShowGridCell()
    rospy.loginfo("ShowCells Node is ready!")
    # In the rospy, there is no rosspinOnce, Because all subscriber is runing
    # in the different thread.We use rospy.is_shutdown() tell tge ctr + C
    while not rospy.is_shutdown():
        if update_gridInfo_flag:
            shHD.showHDCell(curr_coordinate)
            shPosition.showGridCell(curr_coordinate)
            update_gridInfo_flag = False
