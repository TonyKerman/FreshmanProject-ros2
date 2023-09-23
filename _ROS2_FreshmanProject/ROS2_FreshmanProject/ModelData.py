import numpy as np
arm_length = np.array([0.17, 0.14, 0, 0],dtype=float)
servos_bis = np.array([775, 1155, 400, 700],dtype=int)


def angleToPos(angles):
    try:
        angles[0] = -angles[0]
        angles[1] = -angles[1]
        angles[2] = angles[2]
    except IndexError:
        pass
    if type(angles)!=np.ndarray:
        raise TypeError('type wrong')
    length = angles.shape[0]
    pos = np.zeros(length)
    if servos_bis.shape[0] >=length:
        pos=angles/0.24+servos_bis[:length]
    else:
        raise IndexError
    pos = pos.astype(int)
    return pos

