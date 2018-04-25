# Defines constants for the RoboMagellan project so that they can be
# set in one place.


class Topics:
    """Defines topic names for the RoboMagellan project."""

    WAYPOINTS = '/mavros/mission/waypoints'
    PULL_WAYPOINTS = '/mavros/mission/pull'
    PUSH_WAYPOINTS = '/mavros/mission/push'
    SET_CURRENT_WAYPOINT = '/mavros/mission/set_current'
    RC_OVERRIDE = '/mavros/rc/override'
    SETPOINT_VELOCITY = '/mavros/setpoint_velocity/cmd_vel'
    GET_PARAM = '/mavros/param/get'
    SET_PARAM = '/mavros/param/set'
    ARMING = '/mavros/cmd/arming'
    SET_MODE = '/mavros/set_mode'
    ROBOT_POSE = '/mavros/local_position/pose'
    ROBOT_STATE = '/mavros/state'
    CONE_LOCATIONS = '~cone_locations'
    EXEC_CMD = '~exec_cmd'
    TOUCH = '~touch'
    LOCAL_WAYPOINTS = '~waypoints/local'
    UPDATE_WAYPOINTS = '~waypoints/update'
    NAVIGATOR_STATE = '~state'


class ExecCmds:
    """Defines commands for controlling the navigation code."""
    START = 'START_EXEC'
    RESET = 'RESET'
    ADJUST_WAYPOINTS = 'ADJUST_WAYPOINTS'
