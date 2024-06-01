import os
import argparse

import numpy as np
from panda_polymetis.control.panda_gripper_client import PandaGripperClient

parser = argparse.ArgumentParser()
parser.add_argument('--sim', action='store_true')
args = parser.parse_args()

gc = PandaGripperClient(server_ip='localhost', fake=args.sim)

state = gc.get_and_update_state()

print(f"grasp_grip_pos: {state['pos']:.3}")