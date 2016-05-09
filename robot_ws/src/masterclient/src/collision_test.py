import numpy as np
import matplotlib.pyplot as plt
import CollisionAvoidance
"""
k = 0.3
T = 3
c_a = CollisionAvoidance.CollisionAvoidance(0.5**2, 1e-2**2, k)
dsad = False
while not dsad:
    rob1 = [6*np.random.rand(2)-3, 2*np.pi*np.random.rand(), np.random.rand(), 2*np.random.rand()-1]
    rob2 = [6*np.random.rand(2)-3, 2*np.pi*np.random.rand(), np.random.rand(), 2*np.random.rand()-1]
    dsad = c_a.gradient_descent(rob1[0], rob1[1], rob1[2], rob1[3], rob2[0], rob2[1], rob2[2], rob2[3], T)
print dsad
time = np.linspace(0, T, 101)
pos1 = np.zeros((2, 101))
pos2 = np.zeros((2, 101))
for i in range(0, len(time)):
    asd, dsa = CollisionAvoidance.predict(rob1[0], rob1[1], rob1[2], rob1[3], time[i])
    pos1[:, i] = np.array([asd[0, 0], asd[2, 0]])
    asdd, dsad = CollisionAvoidance.predict(rob2[0], rob2[1], rob2[2], rob2[3], time[i])
    pos2[:, i] = np.array([asdd[0, 0], asdd[2, 0]])
plt.plot(pos1[0, :], pos1[1, :], 'g')
plt.plot(pos2[0, :], pos2[1, :], 'b')
plt.figure()

a = np.linspace(-3, 3)
b = np.linspace(-3, 3)
A, B = np.meshgrid(a, b)
# val = __tar_fun_lin_lin__(rob1[0], rob1[1], rob1[2], rob2[0], rob2[1], rob2[2], A, B)
val = CollisionAvoidance.__tar_fun_rot_rot__(rob1[0], rob1[1], rob1[2], rob1[3], rob2[0], rob2[1], rob2[2], rob2[3], A, B)
# val = __tar_fun_lin_rot__(rob1[0], rob1[1], rob1[2], rob2[0], rob2[1], rob2[2], rob2[3], A, B)
# val = __tar_fun_rot_lin__(rob1[0], rob1[1], rob1[2], rob1[3], rob2[0], rob2[1], rob2[2], B, A)
f = 0
g = 0
plt.plot(f, g, 'ro')
for i in range(0, 300):
    # v, u = __tar_grad_lin_lin__(rob1[0], rob1[1], rob1[2], rob2[0], rob2[1], rob2[2], f, g)
    v, u = CollisionAvoidance.__tar_grad_rot_rot__(rob1[0], rob1[1], rob1[2], rob1[3], rob2[0], rob2[1], rob2[2], rob2[3], f, g)
    # v, u = __tar_grad_lin_rot__(rob1[0], rob1[1], rob1[2], rob2[0], rob2[1], rob2[2], rob2[3], f, g)
    # v, u = __tar_grad_rot_lin__(rob1[0], rob1[1], rob1[2], rob1[3], rob2[0], rob2[1], rob2[2], g, f)
    f -= k*v
    g -= k*u
    if 0 > f or f > T or 0 > g or g > T:
        print 'no collision'
        break
    plt.plot(f, g, 'ro')
asd, dsa = CollisionAvoidance.predict(rob1[0], rob1[1], rob1[2], rob1[3], g)
asdd, dsaa = CollisionAvoidance.predict(rob2[0], rob2[1], rob2[2], rob2[3], f)
print asd[0, 0], asd[2, 0], '\n', asdd[0, 0], asdd[2, 0]
CS = plt.contour(A, B, val, 100)
plt.clabel(CS, inline=1, fontsize=10)
plt.title('Simplest default with labels')
plt.show()
"""
"""
pos1 = 2*np.array([[1, 0, -1, 0],
                   [0, 1, 0, -1]])
theta1 = np.array([0, np.pi/2, np.pi, 2*np.pi/2])
pos2 = 2*np.array([[1/np.sqrt(2), -1/np.sqrt(2), -1/np.sqrt(2), 1/np.sqrt(2)],
                   [1/np.sqrt(2), 1/np.sqrt(2), -1/np.sqrt(2), -1/np.sqrt(2)]])
theta2 = 0
x = 1
z = np.array([1, -1])

a = np.linspace(-3, 3)
b = np.linspace(-3, 3)
A, B = np.meshgrid(a, b)
for i in range(0, np.size(pos1, axis=1)):
    for j in range(0, np.size(pos2, axis=1)):
        plt.figure()
        plt.subplot(211)
        plt.plot(pos1[0, i], pos1[1, i], 'go')
        plt.plot(pos2[0, j], pos2[1, j], 'bo')
        plt.axis([-2.5, 2.5, -2.5, 2.5])
        plt.subplot(212)
        val = CollisionAvoidance.__tar_fun_rot_rot__(pos1[:, i], theta1[0], x, z[0], pos2[:, j], theta2, x, z[0], A, B)
        CS = plt.contour(A, B, val, 100)
        plt.clabel(CS, inline=1, fontsize=10)
        plt.title('Simplest default with labels')
        plt.axis('equal')
plt.show()
"""
k = 0.1
T = 0.5
c_a = CollisionAvoidance.CollisionAvoidance(0.6, 5e-2, k)
for i in range(0, 100):
    rob1 = [8*np.random.rand(2)-4, 2*np.pi*np.random.rand(), np.random.rand(), 2*np.random.rand()-1]
    rob2 = [8*np.random.rand(2)-4, 2*np.pi*np.random.rand(), np.random.rand(), 2*np.random.rand()-1]
    if np.linalg.norm(rob2[0]-rob1[0]) > 1.1 * 0.5:
        c_a.calc_new_controls(rob1[0], rob1[1], rob1[2], rob1[3], rob2[0], rob2[1], rob2[2], rob2[3], T)
plt.show()

