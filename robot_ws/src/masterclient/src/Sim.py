import numpy as np
import matplotlib.pyplot as plt
import Robot

sigma_meas = 0.05
n_iter = 50
n_rob = 3
n_iter_no_corr = n_rob-1

x_max = 0.5
x_min = 0.05
z_max = 1
z_min = 0
sigma_x = 0.05
sigma_z = 0.025
x = 0
z = 0
dt = 0.5
k = 0.5
t_x = 2
t_z = 2
ok_dist = 0.05
robot = []
true_robot = []
kalman = []
controls = []
positions = np.array([[1, 4], [4, 2], [3, 1]])
orientat = np.array([0.4, 4, 4])
for i in range(0, n_rob):
    robot += [Robot.Robot(x, z, orientat[i], positions[i])]
    robot[i].set_kalman(sigma_meas, sigma_x, sigma_z)
    robot[i].set_controls(x_min, x_max, z_min, z_max, k, t_x, t_z, ok_dist)
    true_robot += [Robot.Robot(x, z, robot[i].get_theta(), robot[i].get_pos())]
    true_robot[i].set_kalman(sigma_meas, sigma_x, sigma_z)
    true_robot[i].set_controls(x_min, x_max, z_min, z_max, k, t_x, t_z, ok_dist)
positions2 = np.array([[-0.2, -4], [-4, 2]])
base = Robot.Robot(0, 0, 0, positions2[0])
end_node = Robot.Robot(0, 0, 0, positions2[1])
plt.plot(base.get_x_pos(), base.get_y_pos(), 'bo')
plt.plot(end_node.get_x_pos(), end_node.get_y_pos(), 'go')

for j in range(0, n_iter):
    corr_idx = np.mod(j, n_rob)  # Decide which robot should correct its position
    for i in range(0, n_rob):  # Calculate/Estimate new state
        new_state1 = true_robot[i].get_kalman().predict(true_robot[i].get_state(),
                                                       true_robot[i].get_x(), true_robot[i].get_z(), dt)
        true_robot[i].set_state(new_state1)
        if i != corr_idx:
            new_state2 = robot[i].get_kalman().predict(robot[i].get_state(),
                                                       robot[i].get_x(), robot[i].get_z(), dt)
            robot[i].set_state(new_state2)
        else:
            # We should have a method call that measures the robot's position here
            meas_pos = true_robot[i].get_pos()+np.random.normal(0, true_robot[i].get_kalman().get_sigma_meas(), 2)
            new_state2 = robot[i].get_kalman().correct(robot[i].get_state(), meas_pos,
                                                       robot[i].get_x(), robot[i].get_z(), dt)
            robot[i].set_state(new_state2)
            plt.plot(meas_pos[0], meas_pos[1], 'ro')
        plt.plot(robot[i].get_x_pos(), robot[i].get_y_pos(), 'ko')
    for i in range(0, n_rob):  # Calculate new controls at time k
        control_noise_t = np.random.normal(0, robot[i].get_kalman().get_sigma_x())
        control_noise_r = np.random.normal(0, robot[i].get_kalman().get_sigma_z())
        if i != 0 and i != n_rob-1:
            x3, v3 = robot[i].get_controls().calc_controls(robot[i].get_theta(), robot[i].get_pos(),
                                                           robot[i-1].get_pos(), robot[i+1].get_pos())
            robot[i].set_x(x3)
            robot[i].set_z(v3)
            true_robot[i].set_x(x3*(1+control_noise_t))
            true_robot[i].set_z(v3*(1+control_noise_r))
        elif i == 0:
            x3, v3 = robot[i].get_controls().calc_controls(robot[i].get_theta(), robot[i].get_pos(),
                                                           base.get_pos(), robot[i+1].get_pos())
            robot[i].set_x(x3)
            robot[i].set_z(v3)
            true_robot[i].set_x(x3*(1+control_noise_t))
            true_robot[i].set_z(v3*(1+control_noise_r))
        elif i == n_rob-1:
            x3, v3 = robot[i].get_controls().calc_controls(robot[i].get_theta(), robot[i].get_pos(),
                                                           robot[i-1].get_pos(), end_node.get_pos())
            robot[i].set_x(x3)
            robot[i].set_z(v3)
            true_robot[i].set_x(x3*(1+control_noise_t))
            true_robot[i].set_z(v3*(1+control_noise_r))

print true_robot[0].get_theta() - robot[0].get_theta()
print true_robot[1].get_theta() - robot[1].get_theta()
print true_robot[2].get_theta() - robot[2].get_theta()
plt.plot(robot[0].get_pos()[0], robot[0].get_pos()[1], 'yo')
plt.plot(robot[1].get_pos()[0], robot[1].get_pos()[1], 'yo')
plt.plot(robot[2].get_pos()[0], robot[2].get_pos()[1], 'yo')
plt.show()
