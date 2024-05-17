#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
launch_robot.py --config-dir $SCRIPT_DIR/../panda_polymetis/conf/ robot_client=panda_sim use_real_time=false gui=false