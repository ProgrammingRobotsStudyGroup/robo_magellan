# LocalWaypointList.msg
#
#  :current_seq:   seq nr of currently active waypoint
#                  waypoints[current_seq].is_current == True
#
#  :waypoints:     list of waypoints

# The waypoints are in map coordinates.
uint8 FRAME_MAP = 0
# The waypoints are map coordinate offsets relative to the current
# robot position.
uint8 FRAME_ROBOT_RELATIVE = 1

# The frame in which to interpret the waypoints.
uint8 frame

uint16 current_seq
LocalWaypoint[] waypoints
