import matplotlib.pyplot as plt

# new Waypoints from user input
points = [
    (0,0,0),
    (10,0,8),
    (10,10,16),
    (-10,10,24),
    (-10,0,32),
    (-10,-10,40),
    (0,-10,48),
    (10,-10,56),
    (10,0,64),
    (0,0,72),
]
# old Waypoints from user input
# points = [
#     (0,0,0),
#     (10,0,8),
#     (10,5,16),
#     (10,12,24),
#     (0,12,32),
#     (8,12,40),
#     (8,4,48),
#     (0,0,56),
#     (-5,0,64),
#     (-5,-10,72),
#     (-12,-10,80),
#     (0,0,90),
# ]

# Separate x, y, and time
xs = [p[0] for p in points]
ys = [p[1] for p in points]
ts = [p[2] for p in points]

# Plot
plt.figure()
plt.plot(xs, ys, marker='o')

# Annotate time values
for x, y, t in points:
    plt.annotate(f"{t}s", (x, y))

plt.title("Actor Path with Time")
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.grid(True)
plt.axis('equal')
plt.show()
