"""A simmulator for DF algorithms"""
import time
import zmq
import numpy as np

LIGHTSPEED = 299792458

class StateVector:
    """Defines the position and velocity at some time"""

    def __init__(self, position, velocity):
        self.position = np.array(position)
        self.velocity = np.array(velocity)

    def get_distance(self, state_vector):
        """Get the distance between another state vector"""
        dist = 0
        for i, _ in enumerate(self.position):
            dist += (self.position[i] - state_vector.position[i]) ** 2
        return np.sqrt(dist)

class motion:
    """Represents the motion of a radio"""
    def __init__(self, start_time):
        self.start_time = start_time

    def interp_at_time(self) -> StateVector:
        return None

class motion_linear(motion):
    """Represents linear motion"""
    def __init__(self, start_time: float, start: StateVector, end: StateVector, speed: float):
        super().__init__(start_time)
        self.start = start
        self.end = end
        self.speed = speed
        self.velocity = (end.position - start.position) * (speed/start.get_distance(end))
        self.dist = start.get_distance(end)

    def interp_at_time(self, time:float) -> StateVector:
        frac = (time - self.start_time) * self.speed
        pos = self.start + (self.end - self.start) * frac
        return StateVector(pos, self.velocity)


class motion_static(motion):
    """Represents static motion"""
    def __init__(self, start_time:float, start: StateVector):
        super().__init__(start_time)
        self.start = start
    
    def interp_at_time(self, time) -> StateVector:
        return self.start


class Radio:
    """A representation of an SDR with antenna location and power"""

    def __init__(self, power: float,
                 state_vector: StateVector,
                 sps: float,
                 height: float):
        self.power = power
        self.state_vector = state_vector
        self.sps = sps
        self.height = height

    def generate(self, freq: float, t_start: float, t_sec: float):
        """Generate a complex sinusoid for this radio"""
        data = np.linspace(t_start, t_start + t_sec, int(t_sec * self.sps))
        data = np.cos(data*(2*np.pi)*freq) + 1j * np.sin(data*(2*np.pi)*freq)
        return data
    


class RadioTx(Radio):

    def __init__(self, power: float,
                state_vector: StateVector,
                sps: float,
                height: float,
                freq: float):
        super().__init__(power, state_vector, sps, height)
        self.freq = freq
        self.freq_factor = .8 - 1.56 * np.log10(freq)
        self.height_factor = (1.1 * np.log10(freq) - .7)

class RadioRx(Radio):

    def __init__(self, power: float,
                state_vector: StateVector,
                sps: float,
                height: float,
                zmq_address: str,
                zmq_topic: str):
        super().__init__(power, state_vector, sps, height)
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
        self.light_speed = LIGHTSPEED
        self.rx_path_loss = np.ones((len(rx_radio), len(tx_radio)))
        self.noise_power = 8

    def sim(self, t_start: float, t_sec: float):
        """Generates output for each receiver"""
        signals = [radio.generate(radio.freq, t_start, t_sec) for radio in self.tx_radio]
        for radio in self.rx_radio:
            message = np.zeros(signals[0].shape) + 1j*np.zeros(signals[0].shape)
            for i, sig in enumerate(signals):
                dist = radio.state_vector.get_distance(self.tx_radio[i].state_vector)
                phase_shift = dist * self.tx_radio[i].freq * 2 * np.pi / self.light_speed
                message += sig * np.exp(1j * phase_shift)
            radio.recv(message)

    def channel_loss(self):
        """Calculate a channel matrix for future loss calculations, 
            Uses Hata model for medium cities"""
        for i, rx in self.rx_radio:
            for j, tx in self.rx_radio:
                loss = 69.55 + 26.16*np.log10(tx.freq / 1e6) - 13.82 * np.log10(tx.height) - \
                    (tx.freq_factor + tx.height_factor * rx.height) + (44.9 - 6.55 * np.log10(tx.height)) \
                    * tx.state_vector.get_distance(rx.state_vector)
                self.rx_path_loss[i,j] = loss

    def large_scale(self, lenght):
        """Apply large scale fading to a signal"""
        return np.random.normal(loc=0.0, scale=10**(self.noise_power/20), size=lenght) + \
                1j * np.random.normal(loc=0.0, scale=10**(self.noise_power/20), size=lenght)


def main():
    """The main execution"""
    sps = 2.5E6
    tx_pos = StateVector([200000,200000], [0,0])
    rx_pos = [StateVector([(i-1)*1500,0], [0,0]) for i in range(3)]
    rtx = [RadioTx(10, tx_pos, sps, 30, 1e5)]
    rrx = [RadioRx(10, rxp, sps, 1.5, f"tcp://*:6000{i}", b'') for i, rxp in enumerate(rx_pos)]
    antena_array = AntennaArray(rtx, rrx)
    now = time.time()
    rate = 1.0
    while (1):
        if time.time() > now + rate:
            now = time.time()
            antena_array.sim(now, 1)

if __name__ == '__main__':
    main()