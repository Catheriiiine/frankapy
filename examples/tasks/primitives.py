# coding=utf-8
# Copyright 2024 The Ravens Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Motion primitives."""

import numpy as np
from examples.utils import utils
from autolab_core import RigidTransform
from frankapy import FrankaArm

def push(fa, pose0, pose1):  # pylint: disable=unused-argument
  """Execute pushing primitive.

  Args:
    movej: function to move robot joints.
    movep: function to move robot end effector pose.
    ee: robot end effector.
    pose0: SE(3) starting pose.
    pose1: SE(3) ending pose.

  Returns:
    timeout: robot movement timed out if True.
  """
  # reset franka to its home joints
  fa.reset_joints()

  # # read functions
  # T_ee_world = fa.get_pose()
  # print('Translation: {} | Rotation: {}'.format(T_ee_world.translation, T_ee_world.quaternion))

  # pose0 = T_ee_world

  # Adjust push start and end positions.
  pos0 = np.float32((pose0[0][0], pose0[0][1], 0.005))
  pos1 = np.float32((pose1[0][0], pose1[0][1], 0.005))
  vec = np.float32(pos1) - np.float32(pos0)
  length = np.linalg.norm(vec)
  vec = vec / length
  pos0 -= vec * 0.02
  pos1 -= vec * 0.05

# Align spatula against push direction.
  theta = np.arctan2(vec[1], vec[0])
  rot = utils.eulerXYZ_to_quatXYZW((0, 0, theta))

  over0 = (pos0[0], pos0[1], 0.31)
  over1 = (pos1[0], pos1[1], 0.31)

  # # joint controls
  # print('Rotating last joint')
  # joints = fa.get_joints()
  # joints[6] += np.deg2rad(45)
  # fa.goto_joints(joints)
  # joints[6] -= np.deg2rad(45)
  # fa.goto_joints(joints)

# end-effector pose control
  print('Rotation in end-effector frame')
  T1 = RigidTransform(
      translation=over0,
      rotation=rot,
      from_frame='franka_tool', to_frame='franka_tool'
    )
  fa.goto_pose(T1)

  T2 = RigidTransform(translation=pos0, rotation=rot)
  fa.goto_pose(T2)

  print('Translation')
  n_push += (np.int32(np.floor(np.linalg.norm(pos1 - pos0) / 0.01)))
  for _ in range(n_push):
    target = pos0 + vec * n_push * 0.01
    fa.goto_pose(RigidTransform(translation=target, rotation=rot), speed=0.003)
  
  # Move to the end position
  fa.goto_pose(RigidTransform(translation=pos1, rotation=rot), speed=0.003)

  # Move to 'over' position above the end
  fa.goto_pose(RigidTransform(translation=over1, rotation=rot))



class PushContinuous:
  """A continuous pushing primitive."""

  def __init__(self, robot, fast_speed=0.01, slow_speed=0.003):
    self.robot = FrankaArm()
    self.fast_speed = fast_speed
    self.slow_speed = slow_speed

  def reset(self):
    # Reset might be to return the robot to a home or safe pose
    self.robot.reset_joints()
    self.robot.reset_pose()

  def __call__(self, action):
    # Extract pose and slowdown command from action dictionary
    pose = action['move_cmd'][0]
    orientation = action['move_cmd'][1]
    slowdown_cmd = action['slowdown_cmd']

    # Create a RigidTransform from the pose and orientation
    transformation = RigidTransform(translation=np.array(pose), rotation=orientation)

    # Determine speed
    speed = self.slow_speed if slowdown_cmd else self.fast_speed

    # Execute the movement
    timeout = self.robot.goto_pose(transformation, speed=speed)

    return timeout
  
if __name__ == "__main__":
  fa = FrankaArm()
  pose0 = (0, 0)
  pose1 = (0.5, 0.5)
  push(fa, pose0, pose1)
