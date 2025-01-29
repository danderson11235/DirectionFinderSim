"""A simmulation of a rotating DF array"""

import numpy as np
import main

def rotation_matrix(axis, theta):
    axis = np.asarray(axis)
    axis = axis / np.sqrt(np.dot(axis, axis))
    a = np.cos(theta/2.0)
    b, c, d = -axis * np.sin(theta/2.0)
    aa, bb, cc, dd = a*a, b*b, c*c, d*d
    bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
    return np.array([[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac)],
                    [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab)],
                    [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc]])


def rotate_arround_z(vec, theta):
    rot_matrix = rotation_matrix([0,0,1], theta)
    return vec @ rot_matrix

    

def rotationSim():
    sps = 3.5e6
    tx_pos = [(10,10,10), (20,-20,10)]
    rx_pos = [np.array([1,0,0]), np.array([-1,0,0])]
    tx_sv = [main.StateVector(p, (0,0,0)) for p in tx_pos]
    tx_radio = [main.RadioTx(12, xv, sps, 30, 1e5) for xv in tx_sv]
    phi_range = np.linspace(0, np.pi, 100)
    for phi in phi_range:
        cur_rx = [rotate_arround_z(rx, phi) for rx in rx_pos] 
        rx_radio = [main.RadioRxFile(12, main.StateVector(pos, (0,0,0)), sps, 30, f"{phi}.txt") for pos in cur_rx]
        aa = main.AntennaArray(tx_radio, rx_radio)
        aa.sim(0, .001)

def mainfun():
    rotationSim()

if __name__ == "__main__":
    mainfun()
