# This is a message that holds data to describe a information value of a set of torque controlled joints. 
#
# The info of each joint is defined by a real vector
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#


Header header

string[] name
float64[] data
