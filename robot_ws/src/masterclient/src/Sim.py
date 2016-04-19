import numpy as np
import matplotlib.pyplot as plt
import Robot

sigma_meas = 0.05
n_iter = 70
n_rob = 3
n_iter_no_corr = n_rob-1

x_max = 1
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
for i in range(0, n_rob):
    robot += [Robot.Robot(x, z, 2*np.pi*np.random.rand(), 10*np.random.rand(2)-5)]
    robot[i].set_kalman(sigma_meas, sigma_x, sigma_z, dt)
    robot[i].set_controls(x_min, x_max, z_min, z_max, k, t_x, t_z, ok_dist)
    true_robot += [Robot.Robot(x, z, robot[i].get_theta(), robot[i].get_pos())]
    true_robot[i].set_kalman(robot[i].get_kalman().get_sigma_meas(), robot[i].get_kalman().get_sigma_x(),
                             robot[i].get_kalman().get_sigma_z(), robot[i].get_kalman().get_time_step())
    true_robot[i].set_controls(robot[i].get_controls().get_x_min(), robot[i].get_controls().get_x_max(),
                               robot[i].get_controls().get_z_min(), robot[i].get_controls().get_z_max(),
                               robot[i].get_controls().get_k(), robot[i].get_controls().get_t_x(),
                               robot[i].get_controls().get_t_z(), robot[i].get_controls().get_ok_dist())
base = Robot.Robot(0, 0, 0, 10*np.random.rand(2)-5)
end_node = Robot.Robot(0, 0, 0, 10*np.random.rand(2)-5)
plt.plot(base.get_pos()[0], base.get_pos()[1], 'bo')
plt.plot(end_node.get_pos()[0], end_node.get_pos()[1], 'go')

for j in range(0, n_iter):
    corr_idx = np.mod(j, n_rob)  # Decide which robot should correct its position
    for i in range(0, n_rob):  # Calculate/Estimate new state
        x1, v1 = true_robot[i].get_kalman().predict(true_robot[i].get_pos(), true_robot[i].get_theta(),
                                                    true_robot[i].get_x(), true_robot[i].get_z())
        true_robot[i].set_theta(v1)
        true_robot[i].set_pos(np.array([x1[0, 0], x1[2, 0]]))
        if i != corr_idx:
            x2, v2 = robot[i].get_kalman().predict(robot[i].get_pos(), robot[i].get_theta(),
                                                   robot[i].get_x(), robot[i].get_z())
            robot[i].set_theta(v2)
            robot[i].set_pos(np.array([x2[0, 0], x2[2, 0]]))
        else:
            # We should have a method call that measures the robot's position here
            meas_pos = true_robot[i].get_pos()+np.random.normal(0, true_robot[i].get_kalman().get_sigma_meas(), 2)
            x2, v2 = robot[i].get_kalman().correct(robot[i].get_pos(), robot[i].get_theta(),
                                                   meas_pos, robot[i].get_x(), robot[i].get_z())
            robot[i].set_theta(v2)
            robot[i].set_pos(np.array([x2[0, 0], x2[2, 0]]))
            plt.plot(meas_pos[0], meas_pos[1], 'ro')
        plt.plot(robot[i].get_pos()[0], robot[i].get_pos()[1], 'ko')
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

plt.plot(robot[0].get_pos()[0], robot[0].get_pos()[1], 'yo')
plt.plot(robot[1].get_pos()[0], robot[1].get_pos()[1], 'yo')
plt.plot(robot[2].get_pos()[0], robot[2].get_pos()[1], 'yo')
plt.show()
