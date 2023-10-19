"""A simmulator for DF algorithms"""
import time
import zmq
import numpy as np

class StateVector:
    """Defines the position and velociry at some time"""

    def __init__(self, position, velocity):
        self.position = position
        self.velocity = velocity

    def getDistance(self, state_vector):
        dist = 0
        for i, _ in enumerate(self.position):
            dist += (self.position[i] - state_vector.position[i]) ** 2
        return np.sqrt(dist)

class Radio:
    """A representation of an SDR with antenna location and power"""

    def __init__(self, power: float,
                 state_vector: StateVector,
                 sps: float):
        self.power = power
        self.state_vector = state_vector
        self.sps = sps

    def generate(self, freq: float, t_start: float, t_sec: float):
        """Generate a complex sinusoid for this radio"""
        data = np.linspace(t_start, t_start + t_sec, int(t_sec * self.sps))
        data = np.cos(data*(2*np.pi)*freq) + 1j * np.sin(data*(2*np.pi)*freq)
        return data

class RadioTx(Radio):

    def __init__(self, power: float, 
                 state_vector: StateVector,
                sps: float,
                freq: float):
        super().__init__(power, state_vector, sps)
        self.freq = freq

class RadioRx(Radio):

    def __init__(self, power: float, state_vector: StateVector, sps: float, zmq_address: str, zmq_topic: str):
        super().__init__(power, state_vector, sps)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.zmq_topic = zmq_topic
        self.socket.bind(zmq_address)


    def recv(self, message: np.array):
        """Takes in a message and transmits it over zmq to a gnu radio endpoint"""
        message = np.array(message, dtype=np.complex64)
        self.socket.send(message.tobytes())


class AntennaArray():
    """Represents an array of antennas"""

    def __init__(self, tx_radio: list, rx_radio: list):
        self.tx_radio = tx_radio
        self.rx_radio = rx_radio
        self.light_speed = 299792458

    def sim(self, t_start: float, t_sec: float):
        """Generates output for each receiver"""
        signals = [radio.generate(radio.freq, t_start, t_sec) for radio in self.tx_radio]
        for radio in self.rx_radio:
            message = np.zeros(signals[0].shape) + 1j*np.zeros(signals[0].shape)
            for i, sig in enumerate(signals):
                dist = radio.state_vector.getDistance(self.tx_radio[i].state_vector)
                phase_shift = dist * self.tx_radio[i].freq * 2 * np.pi / self.light_speed
                message += sig * np.exp(1j * phase_shift)
            radio.recv(message)


def main():
    """The main execution"""
    sps = 2.5E6
    tx_pos = StateVector([200000,200000], [0,0])
    rx_pos = [StateVector([(i-1)*1500,0], [0,0]) for i in range(3)]
    rtx = [RadioTx(10, tx_pos, sps, 1e5)]
    rrx = [RadioRx(10, rxp, sps, f"tcp://*:6000{i}", b'') for i, rxp in enumerate(rx_pos)]
    antena_array = AntennaArray(rtx, rrx)
    now = time.time()
    rate = 1.0
    while (1):
        if time.time() > now + rate:
            now = time.time()
            antena_array.sim(now, 1)

if __name__ == '__main__':
    main()