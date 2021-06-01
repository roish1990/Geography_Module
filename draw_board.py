import math
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import ConvexHull

a = (1, 3)
b = (5, 6)
c = (6, 5)
d = (2, 2)

points = [c, d, a, b]
x = [c[0], d[0], a[0], b[0]]
id = x.index(min(x))
print(id)
new_points = []
for i in range(len(points)):
    new_points.append(points[((id + i) % len(points))])
print(new_points)

shifted_points = []
for p in new_points:
    shifted_points.append((p[0] - new_points[0][0], p[1] - new_points[0][1]))
print(shifted_points)

beta = math.atan(shifted_points[1][1] / shifted_points[1][0])
beta_d = math.degrees(beta)
alpha = np.pi/2 - beta
print(beta_d)
print(alpha)
print(math.pi)

final_points = []
for p in shifted_points:
    if p[0] == 0:
        final_points.append(p)
    else:
        d = math.sqrt(math.pow(p[0], 2) + math.pow(p[1], 2))
        teta = math.atan(p[1] / p[0])
        final_points.append((d * math.cos(teta + alpha), d * math.sin(teta + alpha)))

print(final_points)

############################################################################################################


def minimum_bounding_rectangle(points):
    """
    Find the smallest bounding rectangle for a set of points.
    Returns a set of points representing the corners of the bounding box.

    :param points: an nx2 matrix of coordinates
    :rval: an nx2 matrix of coordinates
    """
    from scipy.ndimage.interpolation import rotate
    pi2 = np.pi/2.

    # get the convex hull for the points
    hull_points = points[ConvexHull(points).vertices]

    # calculate edge angles
    edges = np.zeros((len(hull_points)-1, 2))
    edges = hull_points[1:] - hull_points[:-1]

    angles = np.zeros((len(edges)))
    angles = np.arctan2(edges[:, 1], edges[:, 0])

    angles = np.abs(np.mod(angles, pi2))
    angles = np.unique(angles)

    # find rotation matrices
    # XXX both work
    rotations = np.vstack([
        np.cos(angles),
        np.cos(angles-pi2),
        np.cos(angles+pi2),
        np.cos(angles)]).T
#     rotations = np.vstack([
#         np.cos(angles),
#         -np.sin(angles),
#         np.sin(angles),
#         np.cos(angles)]).T
    rotations = rotations.reshape((-1, 2, 2))

    # apply rotations to the hull
    rot_points = np.dot(rotations, hull_points.T)

    # find the bounding points
    min_x = np.nanmin(rot_points[:, 0], axis=1)
    max_x = np.nanmax(rot_points[:, 0], axis=1)
    min_y = np.nanmin(rot_points[:, 1], axis=1)
    max_y = np.nanmax(rot_points[:, 1], axis=1)

    # find the box with the best area
    areas = (max_x - min_x) * (max_y - min_y)
    best_idx = np.argmin(areas)

    # return the best box
    x1 = max_x[best_idx]
    x2 = min_x[best_idx]
    y1 = max_y[best_idx]
    y2 = min_y[best_idx]
    r = rotations[best_idx]

    rval = np.zeros((4, 2))
    rval[0] = np.dot([x1, y2], r)
    rval[1] = np.dot([x2, y2], r)
    rval[2] = np.dot([x2, y1], r)
    rval[3] = np.dot([x1, y1], r)

    return rval

points = np.random.rand(12,2)
#points = np.array([[301, 559], [257, 489], [183, 488], [142, 579], [190, 678], [145, 786], [202, 867], [272, 855], [310, 754],
 #   [582, 730], [632, 671], [632, 613], [576, 564], [301, 559]])
plt.scatter(points[:,0], points[:,1])
bbox = minimum_bounding_rectangle(points)
plt.fill(bbox[:,0], bbox[:,1], alpha=0.2)
plt.axis('equal')
plt.show()




