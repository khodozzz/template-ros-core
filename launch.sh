#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
echo "This is an empty launch script. Update it to launch your application."

#roslaunch duckietown_demos indefinite_navigation.launch

roslaunch --wait duckietown_demos indefinite_navigation.launch &
sleep 5
rostopic pub /$VEHICLE_NAME/fsm_node/mode duckietown_msgs/FSMState '{header: {}, state: "LANE_FOLLOWING"}'
