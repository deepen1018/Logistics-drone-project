from aoa_info_drone import ROS_FOR_EXAMPLE

pose = [1, 2, 3]
state_ex = [x, y, z, w]

asd = ROS_FOR_EXAMPLE() asd.pose = []

asd.publish_to_ros(pose, state_ex)
print(asd.pose)