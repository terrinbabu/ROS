# This is a message that holds data to describe the limit state of a set of torque controlled joints. 
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and 
#  * the acceleration of the joint (rad/s or m/s) and 
#  * the jerk of the joint (rad/s or m/s) and 
#  * the snap of the joint (rad/s or m/s) and 
#  * the effort that is applied in the joint (Nm or N).
#  * cartesian linear velocity (m/s)
#  * cartesian rotational velocity (rad/s)
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint limit
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state. 
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.


Header header

float64 cartesian_lin_vel
float64 cartesian_rot_vel
string[] name
float64[] upper_position
float64[] lower_position
float64[] velocity
float64[] acceleration
float64[] jerk
float64[] snap
float64[] effort