import matplotlib.pyplot as plt
plt.gca().set_aspect('equal')
import numpy as np


def forward(x, y, theta, v_l, v_r, t, l):
# straight line
    if (v_l == v_r):
        theta_n = theta
        x_n = x + v_l * t * np.cos(theta)
        y_n = y + v_l * t * np.sin(theta)
        # circular motion
    else:
        # Calculate the radius
        R = l/2.0 * ((v_l + v_r) / (v_r - v_l))
        # computing center of curvature
        ICC_x = x - R * np.sin(theta)
        ICC_y = y + R * np.cos(theta)
        # compute the angular velocity
        omega = (v_r - v_l) / l
        # computing angle change
        dtheta = omega * t
        # forward kinematics for differential drive
        x_n = np.cos(dtheta)*(x-ICC_x) - np.sin(dtheta)*(y-ICC_y) + ICC_x
        y_n = np.sin(dtheta)*(x-ICC_x) + np.cos(dtheta)*(y-ICC_y) + ICC_y
        theta_n = theta + dtheta
    return x_n, y_n, theta_n


# set the distance between the wheels and the initial robot position
l = .33

x, y, theta = 0, 0, 0
# plot the starting position
plt.quiver(x, y, np.cos(theta), np.sin(theta))
print("starting pose: x: %f, y: %f, theta: %f" % (x, y, theta))
# first motion
v_l = -360
v_r = 339.75
t = 0.02

x, y, theta = forward(x, y, theta, v_l, v_r, t, l)
plt.quiver(x, y, np.cos(theta), np.sin(theta))
print("after motion 1: x: %f, y: %f, theta: %f" % (x, y, theta))


plt.xlim([-2, 2.5])
plt.ylim([-2, 3.5])
plt.savefig("poses.png")
plt.show()
