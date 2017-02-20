#!/bin/bash
#
# simple diagnostic to filter mavlink messages with specific msgid from /mavlink/from topic
#
# assumes roscore is already running
# msgid == 180 is the CAMERA_FEEDBACK message
# ref:  http://mavlink.org/messages/ardupilotmega#CAMERA_FEEDBACK
#
# publish matching msgid or '' on topic 'from_filt' 
rosrun topic_tools transform /mavlink/from from_filt std_msgs/String 'str(m.msgid) if m.msgid==180 else ""' &
# msgid == 11 is issued if mode changes
#rosrun topic_tools transform /mavlink/from from_filt std_msgs/String 'str(m.msgid) if m.msgid==11 else ""' &
# echo only the non-empty ones
rostopic echo --filter='m if len(m.data)>0 else ""' from_filt
