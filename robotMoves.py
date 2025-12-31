import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class Leg:
    def __init__(self, side, hip_offset):
        self.side = side  # 'L' or 'R'
        self.hip_offset = np.array(hip_offset)
        self.foot = self.hip_offset.copy()
        self.phase = 0.0
        self.in_swing = False


class SimpleLeggedRobot:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        hip_sep = 0.4
        self.legs = [
            Leg('L', [0.0, hip_sep / 2]),
            Leg('R', [0.0, -hip_sep / 2])
        ]

        self.step_length = 0.4
        self.cycle_time = 1.0
        self.time = 0.0

    def step(self, dt, cmd):
        self.time += dt

        for i, leg in enumerate(self.legs):
            phase = (self.time / self.cycle_time + (0.5 if i == 1 else 0.0)) % 1.0
            prev_in_swing = leg.in_swing
            leg.in_swing = phase >= 0.5

            if leg.in_swing and not prev_in_swing:
                forward_bias = self.step_length
                lateral_bias = 0.0
                rot_bias = 0.0

                if cmd == 'left':
                    if leg.side == 'R':
                        lateral_bias = -0.05
                        rot_bias = 0.03
                elif cmd == 'right':
                    if leg.side == 'L':
                        lateral_bias = 0.05
                        rot_bias = -0.03
                elif cmd == 'stop':
                    forward_bias = 0.0

                dx_body = np.array([forward_bias, lateral_bias + leg.hip_offset[1]])
                c, s = np.cos(self.theta), np.sin(self.theta)
                R = np.array([[c, -s], [s, c]])
                world_dx = R @ dx_body

                leg.foot = np.array([self.x, self.y]) + world_dx
                self.theta += rot_bias

        # Move body toward support centroid
        feet = np.array([leg.foot for leg in self.legs])
        centroid = feet.mean(axis=0)
        move_fraction = 0.15
        delta = centroid - np.array([self.x, self.y])
        self.x += delta[0] * move_fraction
        self.y += delta[1] * move_fraction

    def get_draw_state(self):
        feet = [leg.foot for leg in self.legs]
        return self.x, self.y, self.theta, feet


def run_simulation(commands, cycles_per_command=3, dt=0.05):
    robot = SimpleLeggedRobot()

    timeline = []
    for cmd in commands:
        timeline.extend([cmd] * int(cycles_per_command * robot.cycle_time / dt))

    xs, ys, feet_list = [], [], []

    for step_cmd in timeline:
        robot.step(dt, step_cmd)
        x, y, theta, feet = robot.get_draw_state()
        xs.append(x)
        ys.append(y)
        feet_list.append(np.array(feet))

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_aspect('equal')
    ax.set_xlim(min(xs) - 1, max(xs) + 1)
    ax.set_ylim(min(ys) - 1, max(ys) + 1)
    ax.set_title("Simple Legged Robot Simulation")

    body_point, = ax.plot([], [], 'ko', markersize=10)
    heading_line, = ax.plot([], [], 'k-', linewidth=2)
    left_leg_line, = ax.plot([], [], 'r-', linewidth=3)
    right_leg_line, = ax.plot([], [], 'b-', linewidth=3)

    def init():
        body_point.set_data([], [])
        heading_line.set_data([], [])
        left_leg_line.set_data([], [])
        right_leg_line.set_data([], [])
        return body_point, heading_line, left_leg_line, right_leg_line

    def update(i):
        x, y = xs[i], ys[i]
        feet = feet_list[i]

        body_point.set_data(x, y)

        theta = robot.theta
        hx = [x, x + 0.3 * np.cos(theta)]
        hy = [y, y + 0.3 * np.sin(theta)]
        heading_line.set_data(hx, hy)

        left_leg_line.set_data([x, feet[0, 0]], [y, feet[0, 1]])
        right_leg_line.set_data([x, feet[1, 0]], [y, feet[1, 1]])

        return body_point, heading_line, left_leg_line, right_leg_line

    ani = FuncAnimation(
        fig, update, frames=len(xs),
        init_func=init, interval=50, blit=True
    )

    plt.show()


run_simulation(['forward', 'left', 'right', 'stop'])
