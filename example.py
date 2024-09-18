import numpy as np
from animator import Animator
from differential_drive import DifferentialDrive


# Initial conditions
robot = DifferentialDrive(wheel_base=0.5)
robot2 = DifferentialDrive(wheel_base=0.5)

# Arrays to store simulation data
sim_time = 5
time_step = 0.05
sim_step = int(sim_time / time_step)
time = np.linspace(0, sim_time, sim_step, endpoint=False)
vel = np.linspace(0, 1.0, sim_step)
omega = 1.0 * np.sin(2 * np.pi * 0.5 * time)
omega2 = np.zeros(sim_step)
time_hist = np.zeros(sim_step)
v_hist = np.zeros(sim_step)
omega_hist = np.zeros(sim_step)

# Simulation loop
for step in range(sim_step):
    t = time[step]
    robot.update(vel=vel[step], yaw=omega[step], dt=time_step, log=True)
    robot2.update(vel=vel[step], yaw=omega2[step], dt=time_step, log=True)
robot_history = np.array(robot.history)
robot_history2 = np.array(robot2.history)
trajectory = robot_history[:, [0, 1]]
trajectory2 = robot_history2[:, [0, 1]]

# Generate animation
animator = Animator(num_flame=sim_step, tpv=False)
animator.generate(
    history=robot_history, trajectory=trajectory, history2=robot_history2, trajectory2=trajectory2, filename="tmp"
)
