"""
Embedded Python Blocks:

Each time this file is saved, GRC will instantiate the first class it finds
to get ports and parameters of your block. The arguments to __init__  will
be the parameters. All of them are required to have default values!
"""

import numpy as np
from gnuradio import gr


class blk(gr.sync_block):  # other base classes are basic_block, decim_block, interp_block
    """Embedded Python Block example - a simple multiply const"""

    def __init__(self, input_count=3, sample_rate=35e6, vector_size=180):  # only default arguments here
        """arguments to this function show up as parameters in GRC"""
        gr.sync_block.__init__(
            self,
            name='Capon Beam Former',   # will show up in GRC
            in_sig=[(np.complex64, vector_size) for i in range(input_count)],
            out_sig=[(np.float32, vector_size)]
        )
        # if an attribute with the same name as a parameter is found,
        # a callback is registered (properties work, too).
        self.sample_rate = sample_rate
        self.vector_size = vector_size
        self.input_count = input_count


    def work(self, input_items, output_items):
        """Calculate a angle using capon beam former"""
        angles = np.linspace(-.5, .5, self.vector_size) * np.pi
        for i in range(np.asarray(input_items).shape[1]):
            sig = np.squeeze(np.matrix(np.asanyarray(input_items)[:, i, :]))
            r = sig @ sig.H
            r = np.linalg.inv(r)
            power = []
            for theta in angles:
                # The weight vector simmilar in form to a beamformer generating signal
                a = np.matrix(np.exp(-1j*np.pi*np.arange(self.input_count)*np.sin(theta)))
                # Multiply the wegits with the signal then average result
                power.append(np.mean(20*np.log10(np.abs(1/(np.conj(a) @ r @ a.T)))))
            output_items[0][i] = power
        return len(output_items[0])