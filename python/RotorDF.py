"""A simmulation of a rotating DF array"""

import numpy as np
import main
import matplotlib.pyplot as plt

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

    

class BeamFormer():
    def __init__(self, vector_size=2):
        self.vector_size = vector_size
        pass

    def work(self, input_vector):
        # plt.plot(input_vector)
        # plt.show()
        input_vector = input_vector[:180]
        angles = np.linspace(-.5, .5, 180) *np.pi
        sig = np.squeeze(np.matrix(np.asanyarray(input_vector)))
        r = sig @ sig.H
        r = np.linalg.inv(r)
        power = []
        for theta in angles:
            a = np.matrix(np.exp(-1j*np.pi*np.arange(2)*np.sin(theta)))
            power.append(np.mean(20*np.log10(np.abs(1/(np.conj(a).T @ r @ a)))))
        plt.plot(power)
        plt.show()
        return power


def rotationSim():
    SpunData = []
    sps = 1e9
    tx_pos = [(100,100,100), (200,-300,100)]
    rx_pos = [np.array([1,0,0]), np.array([-1,0,0])]
    tx_sv = [main.StateVector(p, (0,0,0)) for p in tx_pos]
    tx_radio = [main.RadioTx(12, xv, sps, 30, 1e7 + np.random.randint(10)*1e6) for xv in tx_sv]
    phi_range = np.linspace(0, np.pi, 8)
    bf = BeamFormer(2)
    for phi in phi_range:
        cur_rx = [rotate_arround_z(rx, phi) for rx in rx_pos]
        rx_radio = [main.RadioRxFile(12, main.StateVector(pos, (0,0,0)), sps, 30, f"{phi}.txt") for pos in cur_rx]
        aa = main.AntennaArray(tx_radio, rx_radio)
        aa.sim(0, 1e-6)
        plt.plot(np.fromfile(f"{phi}.txt", dtype=np.complex64))
        SpunData.append(bf.work(np.fromfile(f"{phi}.txt",dtype=np.complex64)))


def mainfun():
    rotationSim()
    plt.show()

if __name__ == "__main__":
    mainfun()
