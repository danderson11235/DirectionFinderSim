"""
NON-FUNCTIONING



Embedded Python Blocks:

Each time this file is saved, GRC will instantiate the first class it finds
to get ports and parameters of your block. The arguments to __init__  will
be the parameters. All of them are required to have default values!
"""

import numpy as np
from gnuradio import gr


class blk(gr.sync_block):  # other base classes are basic_block, decim_block, interp_block
    """Embedded Python Block example - a simple multiply const"""

    def __init__(self, input_count=3, sample_rate=35e6, vector_size=180, expected_count=2):  # only default arguments here
        """arguments to this function show up as parameters in GRC"""
        gr.sync_block.__init__(
            self,
            name='Embedded Python Block',   # will show up in GRC
            in_sig=[(np.complex64, vector_size) for i in range(input_count)],
            out_sig=[(np.float32, vector_size)]
        )
        # if an attribute with the same name as a parameter is found,
        # a callback is registered (properties work, too).
        self.sample_rate = sample_rate
        self.vector_size = vector_size
        self.input_count = input_count
        self.expected_count = expected_count


    def work(self, input_items, output_items):
        """Calculate a angle using music algorithm"""
        angles = np.linspace(0, 2, self.vector_size) * np.pi
        for i in range(np.asarray(input_items).shape[1]):
            sig = np.squeeze(np.matrix(np.asanyarray(input_items)[:, i, :]))
            power = []
            r = sig @ sig.H
            eigen_val, eigen_vec = np.linalg.eig(r)
            eigen_val_sort = np.argsort(np.abs(eigen_val))
            eigen_vec = eigen_vec[:, eigen_val_sort]
            eigen_vec_remain = np.asmatrix(np.zeros((self.input_count, self.input_count - self.expected_count), dtype=np.complex128))
            for i in range(self.input_count - self.expected_count):
                eigen_vec_remain[:,i] = eigen_vec[:,i]
            for theta in angles:
                # The weight vector simmilar in form to a beamformer generating signal
                a = np.matrix(np.exp(2*-1j*np.pi*np.arange(self.input_count)*np.sin(theta)))
                # The main music algorithm
                p = 1 / (a.H @ eigen_vec_remain @ eigen_vec_remain.H @ a)
                p = np.abs(p[0,0])
                power.append(10*np.log10(np.abs(p[0,0])))
            output_items[0][i] = power
        return len(output_items[0])