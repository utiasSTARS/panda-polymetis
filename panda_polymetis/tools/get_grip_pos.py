import os
import argparse

import numpy as np
from panda_polymetis.control.panda_gripper_client import PandaGripperClient

parser = argparse.ArgumentParser()
parser.add_argument('--sim', action='store_true')
args = parser.parse_args()


if args.sim:
    gc = PandaGripperClient(server_ip='localhost', fake=True)
else:
    gc = PandaGripperClient(server_ip=os.environ.get('NUC_IP', '192.168.2.100'), fake=True)

state = gc.get_and_update_state()

print(f"grasp_grip_pos: {state['pos']:.3}")