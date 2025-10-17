import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import
FuncAnimation

# Robot Model
class Leg:
  def __init__(self, side, hip_offset):
    self.side = side # 'L' or 'R'
    self.hip_offset = np. array(hip_offset) # relative hip position from body center
    self.foot = self.hip_offset.copy() # foot position in world coords (initially under hip)
    self.phase = 0.0 # gait phase for this leg (0..1), 0 = stance start, 0.5 = swing start
    self.in_swing = False

class SimpleLeggedRobot:
  def __init__(self):
    self.x = 0.0 # world x
    self.y = 0.0 # world y (keep fixed for 2D top-down)
    self.theta = 0.0 # heading angle (radians)
    # two hips left and right
    hip_sep = 0.4
    self.legs = [Leg('L', [-0.0, hip_sep/2]), Leg('R', [0.0, -hip_sep/2])]
    self.step_length = 0.4 # desired step length per swing
    self.step_height = 0.08 # not used for top-down, just visual indicator if desired
    self.cycle_time = 1.0 # seconds per leg half-cycle
    self.time = 0.0

  def step(self, dt, cmd):
    self.time += dt
    for i, leg in enumrate(self.legs):
      phase = (self.time / self.cycle_time + (0.5 if i==1 else 0.0)) % 1.0
      prev_in_swing = leg.in_swing
      leg.in_swing = (phase >= 0.5)
      if leg.in_skwing and not prev_in_swing:
        forward_bias = self.step_length
        lateral_bias = 0.0
        rot_bias = 0.0
        if cmd == 'left':
          if leg.side == 'R':
            forward_bias *= 1.1
            lateral_bias = -0.05
            rot_bias = 0.03
          else:
            forward_bias *= 0.9
            lateral_bias = 0.03
            rot_bias = -0.01
        elif cmd == 'right':
          if leg.side == 'L':
            forward_bias *= 1.1
            lateral_bias = 0.05
            rot_bias = -0.03
          else: 
            forward_bias *= 0.9
            lateral_bias = -0.03
            rot_bias = 0.01
        elif cmd == 'stop':
          forward_bias = 0.0

        dx_body = np.array([forward_bias, lateral_bias + leg.hip_offset[1]])
        c, s = np.cos(self.theta), np.sin(self.theta)
        R = np.array([[c, -s], [s, c]])
        world_dx = R @ dx_body
        leg.foot = np.array([self.x, self.y]) + world_dx
        self.theta += rot_bias

  feet = np.array([leg.foot for leg in self.legs])
  centroid = feet.mean(axis=0)
  move_fraction = 0.15
  delta = centroid - np.array([self.x, self.y])
  self.x += delta[0} * move_fraction
  self.y += delta[1] * move_fraction
  self.x *= 0.9995
  self.y *= 0.9995

def get_draw_state(self):
  feet = [leg..foot for leg in self.legs]
  return (self.x, self.y, self.theta, feet)


