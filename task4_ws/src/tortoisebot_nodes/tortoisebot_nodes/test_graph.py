import numpy as np
import matplotlib.pyplot as plt
from math import pi

# Define velocity functions
def compute_velocities(angularTarget, distDiff):
    angularVel = (2.0 * angularTarget / pi)
    sig = (2 / (1 + np.exp(-5 * distDiff))) - 1
    linearVel = 2 * sig * (1 - np.abs(angularVel))
    return angularVel, linearVel

# Input ranges
angular_target = np.linspace(-pi, pi, 200)
dist_diff = np.linspace(-1.0, 1.0, 200)

# Grid
A, D = np.meshgrid(angular_target, dist_diff)

# Compute velocities over grid
angularVel = (2.0 * A / pi)
sig = (2 / (1 + np.exp(-5 * D))) - 1
linearVel = 2 * sig * (1 - np.abs(angularVel))

# ---- 3D Surface Plot ----
fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(A, D, linearVel, edgecolor='none')
ax.set_title("Linear Velocity Surface")
ax.set_xlabel("Angular Target (rad)")
ax.set_ylabel("Distance Difference")
ax.set_zlabel("Linear Velocity")

# ---- Heatmap ----
plt.figure(figsize=(8, 6))
contour = plt.contourf(A, D, linearVel, levels=50)
plt.title("Linear Velocity Heatmap")
plt.xlabel("Angular Target (rad)")
plt.ylabel("Distance Difference")
plt.colorbar(contour, label="Linear Velocity")

plt.show()
