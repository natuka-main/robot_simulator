import yaml
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

with open("./config.yaml", "r") as yml:
    config = yaml.safe_load(yml)
LENGTH = config["animator"]["length"]
WIDTH = config["animator"]["width"]
BACKTOWHEEL = config["animator"]["back_to_wheel"]
WHEEL_LEN = config["animator"]["wheel_len"]
WHEEL_WIDTH = config["animator"]["wheel_width"]
TREAD = config["animator"]["tread"]
WB = config["animator"]["wheel_base"]
AREA = config["animator"]["area"]


class Animator:
    def __init__(self, num_flame, tpv=False):
        self.offset = LENGTH - WB + WHEEL_LEN / 2
        self.num_flame = num_flame
        self.tpv = tpv  # third person view

    def plot(self, frame, path, steer=0.0, truckcolor="-w"):
        # 前輪中心の座標での各パーツ座標を計算
        x = self.history[:, 0][frame] - self.offset * math.cos(self.history[:, 2][frame])
        y = self.history[:, 1][frame] - self.offset * math.sin(self.history[:, 2][frame])
        yaw = self.history[:, 2][frame]
        caster_angle = self.history[:, 3][frame] + yaw
        outline = np.array(
            [
                [-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                [WIDTH / 2, WIDTH / 2, -WIDTH / 2, -WIDTH / 2, WIDTH / 2],
            ]
        )

        fr_wheel = np.array(
            [
                [WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                [
                    -WHEEL_WIDTH - TREAD,
                    -WHEEL_WIDTH - TREAD,
                    WHEEL_WIDTH - TREAD,
                    WHEEL_WIDTH - TREAD,
                    -WHEEL_WIDTH - TREAD,
                ],
            ]
        )
        c_wheel = np.array(
            [
                [WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                [-WHEEL_WIDTH, -WHEEL_WIDTH, WHEEL_WIDTH, WHEEL_WIDTH, -WHEEL_WIDTH],
            ]
        )

        rr_wheel = np.copy(fr_wheel)
        fl_wheel = np.copy(fr_wheel)
        fl_wheel[1, :] *= -1
        rl_wheel = np.copy(rr_wheel)
        rl_wheel[1, :] *= -1

        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
        Rot2 = np.array([[math.cos(steer), math.sin(steer)], [-math.sin(steer), math.cos(steer)]])
        Rot3 = np.array(
            [[math.cos(caster_angle), math.sin(caster_angle)], [-math.sin(caster_angle), math.cos(caster_angle)]]
        )

        fr_wheel = (fr_wheel.T.dot(Rot2)).T
        fl_wheel = (fl_wheel.T.dot(Rot2)).T
        fr_wheel[0, :] += WB
        fl_wheel[0, :] += WB
        fr_wheel = (fr_wheel.T.dot(Rot1)).T
        fl_wheel = (fl_wheel.T.dot(Rot1)).T
        outline = (outline.T.dot(Rot1)).T
        rr_wheel = (rr_wheel.T.dot(Rot1)).T
        rl_wheel = (rl_wheel.T.dot(Rot1)).T
        c_wheel = (c_wheel.T.dot(Rot3)).T

        outline[0, :] += x
        outline[1, :] += y
        fr_wheel[0, :] += x
        fr_wheel[1, :] += y
        rr_wheel[0, :] += x
        rr_wheel[1, :] += y
        fl_wheel[0, :] += x
        fl_wheel[1, :] += y
        rl_wheel[0, :] += x
        rl_wheel[1, :] += y
        c_wheel[0, :] += x
        c_wheel[1, :] += y

        # 描画されたオブジェクトをリストに追加して返す
        artists = []
        artists.extend(self.ax.plot(np.array(outline[1, :]).flatten(), np.array(outline[0, :]).flatten(), truckcolor))
        artists.extend(
            self.ax.plot(np.array(fr_wheel[1, :]).flatten(), np.array(fr_wheel[0, :]).flatten(), truckcolor)
        )
        artists.extend(
            self.ax.plot(np.array(fl_wheel[1, :]).flatten(), np.array(fl_wheel[0, :]).flatten(), truckcolor)
        )
        artists.extend(self.ax.plot(np.array(c_wheel[1, :]).flatten(), np.array(c_wheel[0, :]).flatten(), truckcolor))
        artists.extend(
            self.ax.plot(self.history[:, 1][:frame], self.history[:, 0][:frame], "b-o", label="Robot path", zorder=0)
        )
        artists.extend(self.ax.plot(path[0], path[1], "r--", label="Reference path", zorder=1))
        self.set_figarea(frame=frame)
        return artists

    def set_figarea(self, frame):
        if self.tpv:
            mergin = 1.2
            min_x = np.min(self.trajectory[:, 0])
            max_x = np.max(self.trajectory[:, 0])
            min_y = np.min(self.trajectory[:, 1])
            max_y = np.max(self.trajectory[:, 1])
            center_x = (min_x + max_x) / 2
            center_y = (min_y + max_y) / 2
            area = mergin * abs(max(abs(min_y), abs(max_y), abs(min_x), abs(max_x))) / 2
            self.ax.set_xlim(-area + center_y, area + center_y)
            self.ax.set_ylim(-area + center_x, area + center_x)
        else:
            self.ax.set_xlim(self.history[:, 1][frame] - AREA, self.history[:, 1][frame] + AREA)
            self.ax.set_ylim(self.history[:, 0][frame] - AREA, self.history[:, 0][frame] + AREA)

    def update(self, frame):
        self.ax.cla()
        artists = self.plot(frame=frame, path=[self.trajectory[:, 1], self.trajectory[:, 0]])
        self.ax.set_aspect("equal")
        self.ax.set_facecolor("black")
        self.ax.invert_xaxis()
        self.ax.legend(loc="upper left")
        self.ax.grid()
        return artists

    def generate(self, history, trajectory, filename, fps=25):
        self.history = history
        self.trajectory = trajectory
        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect("equal")
        ani = FuncAnimation(self.fig, self.update, frames=self.num_flame, blit=True)
        ani.save(str(filename) + ".mp4", writer="ffmpeg", fps=fps)
