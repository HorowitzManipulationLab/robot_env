import time

import numpy as np
import matplotlib.pyplot as plt

import robosuite
from robosuite.controllers import load_composite_controller_config

from scipy.spatial.transform import Rotation as R

robot_name = "Indy7"

# create controller
controller_config = load_composite_controller_config(robot = robot_name, controller="indy7_absolute_pose.json")
# print(controller_config)

env = robosuite.make(
    "Stack",
    robots=[robot_name],
    controller_configs=controller_config,   # arms controlled via OSC, other parts via JOINT_POSITION/JOINT_VELOCITY
    has_renderer=True,                      # on-screen rendering
    render_camera=None,              # visualize the "frontview" camera
    has_offscreen_renderer=True,           # no off-screen rendering
    control_freq=20,                        # 20 hz control for applied actions
    horizon=200,                            # each episode terminates after 200 steps
    use_object_obs=False,                   # no observations needed
    use_camera_obs=True,
)
# reset the environment
env.reset()

"""
possible body names:
'world', 'table', 'left_eef_target', 'right_eef_target', 
'robot0_base', 'robot0_link0', 'robot0_link1', 'robot0_link2', 'robot0_link3', 'robot0_link4', 'robot0_link5', 'robot0_link6', 'robot0_link7', 
'robot0_right_hand', 'gripper0_right_right_gripper', 'gripper0_right_eef', 'gripper0_right_leftfinger', 'gripper0_right_finger_joint1_tip', 
'gripper0_right_rightfinger', 'gripper0_right_finger_joint2_tip', 'fixed_mount0_base', 'fixed_mount0_controller_box', 
'fixed_mount0_pedestal_feet', 'fixed_mount0_torso', 'fixed_mount0_pedestal', 'cubeA_main', 'cubeB_main'
"""
cubeA_main_id = env.sim.model.body_name2id("cubeA_main")
pos_cubeA = env.sim.data.body_xpos[cubeA_main_id]
print("Cube A position: ", pos_cubeA)
rotm_cubeA = env.sim.data.body_xmat[cubeA_main_id].reshape((3,3)) # rotation matrix
quat_cubeA = env.sim.data.body_xquat[cubeA_main_id] # quaternion in wxyz format
rotm_from_quat_cubeA = R.from_quat(quat_cubeA, scalar_first = True).as_matrix() # rotation matrix from quaternion

print("confirm both are the same:", rotm_cubeA, rotm_from_quat_cubeA)

print(env.action_spec[0].shape)
rotm_basic = np.array([[
    [0, 1, 0],
    [1, 0, 0],
    [0, 0, -1]
]])

for i in range(1000):
    action = np.zeros((env.action_spec[0].shape[0],))
    action[0:3] = np.array([0.5, 0, 0.3]) # Desired position to go in meter
    action[3:6] = R.from_matrix(rotm_basic).as_rotvec() # Desired orientation to go in rotation vector representation
    action[6] = -1

    obs, reward, done, info = env.step(action)  # take action in the environment

    time.sleep(0.05)

    env.render()  # render on display
    if i == 0:
        for key in obs.keys():
            try:
                print(key, obs[key].shape)
            except:
                print(key, obs[key])
        time.sleep(3)

eef_quat = obs['robot0_eef_quat'] # quaternion in xyzw format
# if you want to convert it into the rotation matrix
eef_rotm = R.from_quat(eef_quat, scalar_first = False).as_matrix()

print(obs['robot0_eef_quat'])

# Show the image, depth, segmentation
# for cam in CAMERA_NAMES:
#     # plt.figure()
#     fig, axes = plt.subplots(1, 3)
#     axes[0].imshow(np.rot90(obs[cam + "_image"], 2))
#     axes[1].imshow(np.rot90(obs[cam + "_depth"], 2), cmap='gray')
#     axes[2].imshow(np.rot90(obs[cam + "_segmentation_instance"], 2), cmap='grey')
#     for ax in axes:
#         ax.axis('off')
#     fig.suptitle(cam)
# plt.show()
