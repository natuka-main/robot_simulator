import yaml
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

with open("./config.yaml", "r") as yml:
    config = yaml.safe_load(yml)

LENGTH = config["animator"]["length"]  # 車両の全長
WIDTH = config["animator"]["width"]  # 車両の幅
BACK_TO_WHEEL = config["animator"]["back_to_wheel"]  # 車体後端から後輪中心までの距離
WHEEL_LEN = config["animator"]["wheel_len"]  # タイヤの長さ
WHEEL_WIDTH = config["animator"]["wheel_width"]  # タイヤの幅
TREAD = config["animator"]["tread"]  # トレッド（左右タイヤ間距離）
WB = config["animator"]["wheel_base"]  # ホイールベース（前後タイヤ間距離）
AREA = config["animator"]["area"]  # 表示エリアのスケール範囲


# アニメーション描画用クラス
class Animator:
    def __init__(self, num_flame, tpv=False, hold=False):
        # 車両の描画中心からのオフセットを設定（後輪中心 → 車体中心）
        self.offset = LENGTH - WB + WHEEL_LEN / 2
        self.num_flame = num_flame  # アニメーションのフレーム数
        self.tpv = tpv  # third person view（全体表示）を使うかどうか

    # 1フレーム分の車両と軌跡の描画を行う関数
    def plot(self, frame, state, path, steer=0.0, truckcolor="-w"):
        # 車体中心の座標を算出（後輪中心からオフセット）
        x = state[:, 0][frame] - self.offset * math.cos(state[:, 2][frame])
        y = state[:, 1][frame] - self.offset * math.sin(state[:, 2][frame])
        yaw = state[:, 2][frame]  # 車体の向き（ヨー角）
        caster_angle = state[:, 3][frame] + yaw  # キャスター角（補助輪角）

        # 車両本体のポリゴン形状を定義（ローカル座標）
        outline = np.array(
            [
                [-BACK_TO_WHEEL, (LENGTH - BACK_TO_WHEEL), (LENGTH - BACK_TO_WHEEL), -BACK_TO_WHEEL, -BACK_TO_WHEEL],
                [WIDTH / 2, WIDTH / 2, -WIDTH / 2, -WIDTH / 2, WIDTH / 2],
            ]
        )

        # 前輪・補助輪などの形状を定義（ローカル座標）
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

        # 各タイヤを左右反転などで生成
        fl_wheel = np.copy(fr_wheel)
        fl_wheel[1, :] *= -1

        # 回転行列を定義（車体・前輪・キャスター）
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
        Rot2 = np.array([[math.cos(steer), math.sin(steer)], [-math.sin(steer), math.cos(steer)]])
        Rot3 = np.array(
            [[math.cos(caster_angle), math.sin(caster_angle)], [-math.sin(caster_angle), math.cos(caster_angle)]]
        )

        # 前輪にステア角を適用し、前方に移動
        fr_wheel = (fr_wheel.T.dot(Rot2)).T
        fl_wheel = (fl_wheel.T.dot(Rot2)).T
        fr_wheel[0, :] += WB
        fl_wheel[0, :] += WB

        # 各部位を車体角で回転させる
        fr_wheel = (fr_wheel.T.dot(Rot1)).T
        fl_wheel = (fl_wheel.T.dot(Rot1)).T
        outline = (outline.T.dot(Rot1)).T
        c_wheel = (c_wheel.T.dot(Rot3)).T

        # グローバル座標系に平行移動
        outline[0, :] += x
        outline[1, :] += y
        fr_wheel[0, :] += x
        fr_wheel[1, :] += y
        fl_wheel[0, :] += x
        fl_wheel[1, :] += y
        c_wheel[0, :] += x
        c_wheel[1, :] += y

        # 描画オブジェクトの作成
        artists = []
        artists.extend(self.ax.plot(np.array(outline[1, :]).flatten(), np.array(outline[0, :]).flatten(), truckcolor))
        artists.extend(
            self.ax.plot(np.array(fr_wheel[1, :]).flatten(), np.array(fr_wheel[0, :]).flatten(), truckcolor)
        )
        artists.extend(
            self.ax.plot(np.array(fl_wheel[1, :]).flatten(), np.array(fl_wheel[0, :]).flatten(), truckcolor)
        )
        artists.extend(self.ax.plot(np.array(c_wheel[1, :]).flatten(), np.array(c_wheel[0, :]).flatten(), truckcolor))

        # ロボットの通過軌跡を青線で描画
        artists.extend(self.ax.plot(state[:, 1][:frame], state[:, 0][:frame], "b-o", label="Robot path", zorder=0))

        # 目標経路（赤破線）
        artists.extend(self.ax.plot(path[0], path[1], "r--", label="Reference path", zorder=1))

        # 表示範囲の調整
        self.set_figarea(frame=frame, state=state)

        return artists  # アニメーション描画オブジェクトを返す

    # 表示領域の設定
    def set_figarea(self, frame, state):
        if self.tpv:
            # third-person view（全体表示）モード
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
            # 常に現在位置中心で固定表示
            self.ax.set_xlim(state[:, 1][frame] - AREA, state[:, 1][frame] + AREA)
            self.ax.set_ylim(state[:, 0][frame] - AREA, state[:, 0][frame] + AREA)

    # アニメーション更新関数（1フレーム毎に呼び出される）
    def update(self, frame):
        self.ax.cla()  # 現在の図をクリア
        artists = self.plot(frame=frame, state=self.robot_state, path=[self.trajectory[:, 1], self.trajectory[:, 0]])

        # 比較用の第2経路がある場合も描画
        if not self.trajectory2 is None:
            artists = self.plot(
                frame=frame,
                state=self.robot_state2,
                path=[self.trajectory2[:, 1], self.trajectory2[:, 0]],
                truckcolor="-c",
            )

        # 描画の共通設定
        self.ax.set_aspect("equal")
        self.ax.set_facecolor("black")
        self.ax.invert_xaxis()  # X軸反転でROS座標へ合わせる
        self.ax.legend(loc="upper left")
        self.ax.grid()
        return artists

    # アニメーションの生成とMP4保存
    def generate(self, history, trajectory, filename, history2=None, trajectory2=None, fps=25):
        self.robot_state = history  # メインの車両軌跡
        self.trajectory = trajectory  # 目標経路
        self.robot_state2 = history2 if not history2 is None else None  # 第2車両の軌跡（任意）
        self.trajectory2 = trajectory2 if not trajectory2 is None else None

        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect("equal")

        # アニメーション作成（FuncAnimation）
        ani = FuncAnimation(self.fig, self.update, frames=self.num_flame, blit=True)

        # MP4として保存
        ani.save(str(filename) + ".mp4", writer="ffmpeg", fps=fps)
