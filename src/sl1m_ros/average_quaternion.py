import numpy as np


def normalize(v):
    norm=np.linalg.norm(v)
    if norm==0:
        norm=np.finfo(v.dtype).eps
    return v/norm


def average_quaternion(quats, weights=None):
        if weights is not None:
            assert len(quats) == len(weights)
        else:
            weights = len(quats) * [1]
        np_quats = np.array(quats)
        # Form the symmetric accumulator matrix
        A = np.zeros((4, 4))
        M = np_quats.shape[0]
        wSum = 0

        for i in range(M):
            q = np_quats[i, :]
            w_i = weights[i]
            A += w_i * (np.outer(q, q)) # rank 1 update
            wSum += w_i

        # scale
        A /= wSum

        # Get the eigenvector corresponding to largest eigen value
        q = normalize(np.linalg.eigh(A)[1][:, -1])

        if q[-1] < 0:
            return -q
        else:
            return q
