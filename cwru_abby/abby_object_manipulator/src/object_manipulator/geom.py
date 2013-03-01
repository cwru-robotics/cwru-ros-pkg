import numpy as np
from tf import transformations

def to_hom(points):
    if len(points.shape) == 1:
        points_hom = np.ones(points.shape[0] + 1,)
        points_hom[:-1] = points
    else:
        points_hom = np.ones((points.shape[0], points.shape[1]+1))
        points_hom[:,:-1] = points
    return points_hom

def from_hom(points):
    if len(points.shape) == 1:
        return points[:-1] / points[-1]
    else:
        return points[:,:-1] / np.tile(np.reshape(points[:,-1], (len(points), 1)), (1, points.shape[1]-1))

def transform_points(points, transform):
    '''
    Transform one or more D dimensional points by the given homogeneous transform.

    points - (D,) or (N, D) array

    transform - (D + 1, D + 1) array
    '''
    return from_hom(np.dot(to_hom(points), np.transpose(np.array(transform))))

def transform_vectors(vectors, transform):
    R = np.array(transform[:3,:3]) / transform[3,3]
    return np.dot(vectors, np.transpose(R))

def crop_to_bbox_mask(points, xmin, xmax, ymin, ymax, zmin, zmax):
    bbox_mask = (points[:,0] > xmin) & (points[:,0] < xmax)
    bbox_mask &= (points[:,1] > ymin) & (points[:,1] < ymax)
    bbox_mask &= (points[:,2] > zmin) & (points[:,2] < zmax)
    return bbox_mask

def crop_to_bbox(points, xmin, xmax, ymin, ymax, zmin, zmax):
    return points[crop_to_bbox_mask(points, xmin, xmax, ymin, ymax, zmin, zmax)]

def matrix_to_rot_trans(transform):
    '''
    Converts a 4x4 homogenous rigid transformation matrix to a translation and a
    quaternion rotation.
    '''
    scale, shear, angles, trans, persp = transformations.decompose_matrix(transform)
    rot = transformations.quaternion_from_euler(*angles)
    return rot, trans

def rot_trans_to_matrix(rot, trans):
    '''Converts a rotation and translation to a homogeneous transform.

    Args:
        rot (np.array): Quaternion (x, y, z, w).
        trans (np.array): Translation (x, y, z).

    Returns:
        H (np.array): 4x4 homogenous transform matrix.
    '''
    H = transformations.quaternion_matrix(rot)
    H[0:3, 3] = trans
    return H
