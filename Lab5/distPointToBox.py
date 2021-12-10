import numpy as np
np.seterr(divide='ignore', invalid='ignore')

def distPointToBox(p, box):
    """
    Calculate the distance between a point and an axis-aligned box.
    :param p: Nx3 vector of points [x,y,z]
    :param box: 1x6 vector of minimum and maximum points of box
    :return
    dist - Nx1 vector of distance between the points and the box
            dist > 0 point outside
            dist = 0 point is on or inside box
    unit - Nx3 vector where each row is the corresponding unit vector to the closest spot on the box
        norm(unit) = 1 point is outside the box
        norm(unit)= 0 point is on/inside the box

     Method from MultiRRomero
     @ https://stackoverflow.com/questions/5254838/calculating-distance-between-a-point-and-a-rectangular-box-nearest-point
    """
    # Get box info
    boxMin = np.array([box[0,0], box[0,1], box[0,2]])
    boxMax = np.array([box[0,3], box[0,4], box[0,5]])
    boxCenter = boxMin*0.5 + boxMax*0.5
    p = np.array(p)

    # Get distance info from point to box boundary
    dx = np.amax(np.vstack([boxMin[0] - p[:, 0], p[:, 0] - boxMax[0], np.zeros(p[:, 0].shape)]).T, 1)
    dy = np.amax(np.vstack([boxMin[1] - p[:, 1], p[:, 1] - boxMax[1], np.zeros(p[:, 1].shape)]).T, 1)
    dz = np.amax(np.vstack([boxMin[2] - p[:, 2], p[:, 2] - boxMax[2], np.zeros(p[:, 2].shape)]).T, 1)

    # convert to distance
    distances = np.vstack([dx, dy, dz]).T
    dist = np.linalg.norm(distances, axis=1)

    # Figure out the signs
    signs = np.sign(boxCenter-p)

    # Calculate unit vector and replace with
    unit = distances / dist[:, np.newaxis] * signs
    unit[np.isnan(unit)] = 0
    unit[np.isinf(unit)] = 0
    return dist, unit


if __name__=='__main__':
    box = [-1, -1, -1, 1, 1, 1]
    p = [[0, 0, 0], [1, 1, 1], [2, 2, 2], [3, 3, 3], [-3, -3, -3]]
    dist, unit = distPointToBox(p, box)
    print(unit)
    print(dist)


