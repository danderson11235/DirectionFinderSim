import adi
import numpy as np
import pyqtgraph as pg
import matplotlib.pyplot as plt
import serial
import ArduinoDriver as  ad

samp_rate = 30e6
num_samples = 2**12
rx_lo_low = 2.4e9
rx_lo = 2.4e9
rx_lo_high = 2.85e9
rx_mode = "manual"
rx_gain0 = 40
rx_gain1 = 40
tx_lo_low = rx_lo_low
tx_lo = rx_lo_low
tx_lo_high = rx_lo_high
tx_gain = -3
fc0 = int(200e3)
phase_cal = 0
tracking_length = 1000

rx_phase_offset1 = 52.9170966 * np.pi / 180.0

d_wavelength = 0.0625 # Distance between elements in meters
wavelength = 3E8/rx_lo              # wavelength of the RF carrier
d = d_wavelength*wavelength         # distance between elements in meters

'''The ADALM-Pluto'''
sdr = adi.ad9361(uri='ip:192.168.2.1')

'''Configure radio'''
sdr.rx_enabled_channels = [0,1]
sdr.sample_rate = int(samp_rate)
sdr.rx_rf_bandwidth = int(fc0*3)
sdr.rx_lo = int(rx_lo_low)
sdr.gain_control_mode = rx_mode
sdr.rx_hardwaregain_chan0 = int(rx_gain0)
sdr.rx_hardwaregain_chan1 = int(rx_gain1)
sdr.rx_buffer_size = int(num_samples)
sdr._rxadc.set_kernel_buffers_count(1)   # set buffers to 1 (instead of the default 4) to avoid stale data on Pluto
sdr.tx_rf_bandwidth = int(fc0*3)
sdr.tx_lo = int(tx_lo_low)
sdr.tx_cyclic_buffer = True
sdr.tx_hardwaregain_chan0 = int(tx_gain)
sdr.tx_hardwaregain_chan1 = int(-88)
sdr.tx_buffer_size = int(2**18)

'''Program Tx and Send Data'''
fs = int(sdr.sample_rate)
N = 2**16
ts = 1 / float(fs)
t = np.arange(0, N * ts, ts)
i0 = np.cos(2 * np.pi * t * fc0) * 2 ** 14
q0 = np.sin(2 * np.pi * t * fc0) * 2 ** 14
iq0 = i0 + 1j * q0
sdr.tx([iq0,iq0])  # Send Tx data.

# Assign frequency bins and "zoom in" to the fc0 signal on those frequency bins
xf = np.fft.fftfreq(num_samples, ts)
xf = np.fft.fftshift(xf)/1e6
signal_start = int(num_samples*(samp_rate/2+fc0/2)/samp_rate)
signal_end = int(num_samples*(samp_rate/2+fc0*2)/samp_rate)

'''Configure arduino'''
arduino = ad.SerialDevice("COM5")
angle_low = -90
angle_high = 90

def calcTheta(phase):
    # calculates the steering angle for a given phase delta (phase is in deg)
    # steering angle is theta = arcsin(c*deltaphase/(2*pi*f*d)
    arcsin_arg = np.deg2rad(phase)*3E8/(2*np.pi*rx_lo*d)
    arcsin_arg = max(min(1, arcsin_arg), -1)     # arcsin argument must be between 1 and -1, or numpy will throw a warning
    calc_theta = np.rad2deg(np.arcsin(arcsin_arg))
    return calc_theta

def dbfs(raw_data):
    # function to convert IQ samples to FFT plot, scaled in dBFS
    NumSamples = len(raw_data)
    win = np.hamming(NumSamples)
    y = raw_data * win
    s_fft = np.fft.fft(y) / np.sum(win)
    s_shift = np.fft.fftshift(s_fft)
    s_dbfs = 20*np.log10(np.abs(s_shift)/(2**11))     # Pluto is a signed 12 bit ADC, so use 2^11 to convert to dBFS
    return s_shift, s_dbfs

def monopulse_angle(array1, array2):
    ''' Correlate the sum and delta signals  '''
    # Since our signals are closely aligned in time, we can just return the 'valid' case where the signals completley overlap
    # We can do correlation in the time domain (probably faster) or the freq domain
    # In the time domain, it would just be this:
    # sum_delta_correlation = np.correlate(delayed_sum, delayed_delta, 'valid')
    # But I like the freq domain, because then I can focus just on the fc0 signal of interest
    sum_delta_correlation = np.correlate(array1[signal_start:signal_end], array2[signal_start:signal_end], 'valid')
    angle_diff = np.angle(sum_delta_correlation)
    return angle_diff

def scan_for_DOA():
    # go through all the possible phase shifts and find the peak, that will be the DOA (direction of arrival) aka steer_angle
    data = sdr.rx()
    Rx_0=data[0]
    Rx_1=data[1]
    peak_sum = []
    peak_delta = []
    monopulse_phase = []
    delay_phases = np.arange(-180, 180, 2)    # phase delay in degrees
    for phase_delay in delay_phases:   
        delayed_Rx_1 = Rx_1 * np.exp(1j*np.deg2rad(phase_delay+phase_cal))
        delayed_sum = Rx_0 + delayed_Rx_1
        delayed_delta = Rx_0 - delayed_Rx_1
        delayed_sum_fft, delayed_sum_dbfs = dbfs(delayed_sum)
        delayed_delta_fft, delayed_delta_dbfs = dbfs(delayed_delta)
        mono_angle = monopulse_angle(delayed_sum_fft, delayed_delta_fft)
        
        peak_sum.append(np.max(delayed_sum_dbfs))
        peak_delta.append(np.max(delayed_delta_dbfs))
        monopulse_phase.append(np.sign(mono_angle))
        
    peak_dbfs = np.max(peak_sum)
    peak_delay_index = np.where(peak_sum==peak_dbfs)
    peak_delay = delay_phases[peak_delay_index[0][0]]
    steer_angle = int(calcTheta(peak_delay))
    
    return delay_phases, peak_dbfs, peak_delay, steer_angle, peak_sum, peak_delta, monopulse_phase

def Tracking(last_delay):
    # last delay is the peak_delay (in deg) from the last buffer of data collected
    data = sdr.rx()
    Rx_0=data[0]
    Rx_1=data[1]
    delayed_Rx_1 = Rx_1 * np.exp(1j*np.deg2rad(last_delay+phase_cal))
    delayed_sum = Rx_0 + delayed_Rx_1
    delayed_delta = Rx_0 - delayed_Rx_1
    delayed_sum_fft, delayed_sum_dbfs = dbfs(delayed_sum)
    delayed_delta_fft, delayed_delta_dbfs = dbfs(delayed_delta)
    mono_angle = monopulse_angle(delayed_sum_fft, delayed_delta_fft)
    phase_step= 1
    if np.sign(mono_angle) > 0:
        new_delay = last_delay - phase_step
    else:
        new_delay = last_delay + phase_step
    return new_delay

'''Collect Data'''
for i in range(20):  
    # let Pluto run for a bit, to do all its calibrations
    data = sdr.rx()
    

    
#scan once to get the direction of arrival (steer_angle) as the initial point for out monopulse tracker
delay_phases, peak_dbfs, peak_delay, steer_angle, peak_sum, peak_delta, monopulse_phase = scan_for_DOA()
delay = peak_delay  # this will be the starting point if we are doing monopulse tracking
tracking_angles = np.ones(tracking_length)*180
tracking_angles[:-1] = -180   # make a line across the plot when tracking begins


# Get current angle from the arduino driver

# Rotate every angle at rate wz
# sec 3.2 gives fz = 1/T and T/2 > dt >> T then dt = T/ 2M 
# fz << fs / 2L

# For each angle calculate the phase offset
# 

def plotRotor(angles, results):
    dimm = results.shape[1]
    dimm2 = dimm**2/4
    np.zeros((dimm, dimm))
    for y in (np.arange(dimm)-dimm//2):
        for x in (np.arange(dimm)-dimm//2):
            if (y**2 + x**2) > dimm2:
                continue
            pt = x + 1j*y
            theta = np.arctan2(y, x)
            radius = np.sqrt(y**2 + x**2)

def plotRect(angles, results):
    plt.imshow(results)
    plt.show()


def getX():
    return np.matrix(sdr.rx())

def w_mvdr(theta, X):
    s = np.exp(-2j * np.pi * .5 * np.arange(2) * np.sin(theta) + rx_phase_offset1)
    s = np.matrix(s.reshape(-1,1))
    R = (X @ X.H) / X.shape[1]
    Rinv = np.matrix(np.linalg.pinv(R))
    w = (Rinv @ s)/(s.H @ Rinv @ s)
    return w

def dfGenCapon(resolution):
    res = []
    thetaScan = np.linspace(-1*np.pi, np.pi, resolution)
    X = getX()
    for theta in thetaScan:
        w = w_mvdr(theta, X)
        X_weight = w.H @ X
        power = 10*np.log10(np.var(X_weight))
        res.append(power)
    return np.array(res)



def rotorDF(dfResolution=90, angleResolution=1, angleMin=-90, angleMax=90):
    # Version 2 rotor df at each angle
    # init
    arduino.reset()
    arduino.setDirection(ad.CLOCKWISE)
    # set to -90 to prep for rotation
    arduino.gotoBoundAngle(angleMin)
    arduino.setDirection(ad.WITTERSHINS)

    # for each angle rotate arm and record snapshot
    try:
        angles = np.arange(angleMin, angleMax+1, angleResolution)
        dfResults = np.zeros((len(angles), dfResolution))
        for angle, dfRes in zip(angles, dfResults):
            dfRes += dfGenCapon(dfResolution)
            arduino.gotoBoundAngle(angle)

        arduino.gotoBoundAngle(0)
        return angles, dfResults
    except:
        arduino.gotoBoundAngle(0)
        return np.array([0]), np.array([0])

# angles, dfResults = rotorDF()
# plotRect(angles, dfResults)
# np.save('df_1_-90_90(2).npy', dfResults)