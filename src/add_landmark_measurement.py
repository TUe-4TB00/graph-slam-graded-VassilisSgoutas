import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_landmark_measurement(graph, initial_estimate, result):

    pose4 = result.atPose2(X(4))
    l2 = result.atPoint2(L(2))

    dx = l2[0] - pose4.x()
    dy= l2[1] - pose4.y()

    distance = math.sqrt(dx**2 + dy**2)


    global_angle = math.atan2(dy, dx)
    relative_bearing = global_angle - pose4.theta()

    rotation = math.degrees(relative_bearing)

    # Determine the correct rotation (bearing) and distance from X(4) to L(2) 
    # rotation = 
    # distance = 
    graph.add(gtsam.BearingRangeFactor2D(X(4), L(2), gtsam.Rot2.fromDegrees(rotation), distance, MEASUREMENT_NOISE))

    return graph