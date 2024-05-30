import os

from panda_polymetis.control.panda_client import PandaClient
from transform_utils.pose_transforms import PoseTransformer


# modify this pose!
# move_pose_euler_sxyz = [0.1653715, 0.4675048, 0.5384299, -1.623644, -0.8070195, 0.4270715]
# move_pose_euler_sxyz = [0.2770223, 0.2919437, 0.5384299, -1.623644, -0.8070195, 0.4270715]
move_pose_euler_sxyz = [0.3654211, 0.1962629, 0.682987, 1.91674, -0.7635592, 1.304907]

move_pose = PoseTransformer(pose=move_pose_euler_sxyz, rotation_representation='euler', axes='sxyz')
pc = PandaClient(server_ip=os.environ['NUC_IP'])
pc.move_EE_to(move_pose, time_to_go=4)

pose, pose_arr = pc.get_and_update_ee_pose()
print(f"Pose after move: {pose}")