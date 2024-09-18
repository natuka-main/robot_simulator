import numpy as np


class DifferentialDrive:
    def __init__(self, wheel_base, wheel_radius=None, tread=None, initial_state=None):
        self.wheel_base = wheel_base
        if wheel_radius is not None:
            self.wheel_radius = wheel_radius
        if tread is not None:
            self.tread = tread

        if initial_state is None:
            self.state = np.zeros(4)
        else:
            self.state = np.array(initial_state)

        self.history = []

    def initialize(self, initial_state=None):
        if initial_state is None:
            self.state = np.zeros(4)
        else:
            self.state = np.array(initial_state)
        self.history = []

    def update(self, vel, yaw, dt, log=False):
        dx = vel * np.cos(self.state[2]) * dt
        dy = vel * np.sin(self.state[2]) * dt
        dtheta = yaw * dt

        # Update the state
        self.state[0] += dx
        self.state[1] += dy
        self.state[2] = (self.state[2] + dtheta) % (2 * np.pi)
        self.state[3] = -np.arctan2((self.wheel_base * yaw), vel)
        # Nomrmalize angle (-pi~pi)
        if self.state[2] > np.pi:
            self.state[2] -= 2 * np.pi

        # Add the current state to the state history
        if log:
            self.history.append(self.state.copy())


# Usage
if False:
    import matplotlib.pyplot as plt

    # DifferentialDriveクラスのインスタンスを作成
    robot = DifferentialDrive(wheel_base=0.5)

    # シミュレーションの設定
    total_time = 10.0  # シミュレーションの合計時間（秒）
    dt = 0.1  # シミュレーションの時間ステップ（秒）

    # ロボットの軌跡を保存するリスト
    robot_trajectory = []

    # シミュレーションループ
    num_steps = int(total_time / dt)
    for step in range(num_steps):
        robot.update(vel=1.0, yaw=0.2, dt=dt, log=True)

    # ロボットの軌跡をプロット
    robot_trajectory = np.array(robot.history)
    plt.figure()
    plt.plot(robot_trajectory[:, 0], robot_trajectory[:, 1])
    plt.xlabel("pos X [m]")
    plt.ylabel("pos Y [m]")
    plt.title("trajectory")
    plt.grid(True)
    plt.show()
