# This message describes the state of an actuated joints.
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the joint name
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s), and
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name


# Name of the joint
string name

# States of the joint
float64 position
float64 velocity
float64 error_w1
float64 error_w2
float64 obs_position
float64 obs_velocity
float64 w11
float64 w12
float64 w21
float64 w22
float64 z0
float64 z1
float64 reference
