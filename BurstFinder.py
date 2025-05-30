import adi
import time
import numpy as np
import pyqtgraph as pg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
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
tx_gain_0 = -30
tx_gain_1 = -30
fc0 = int(200e3)
phase_cal = 0
tracking_length = 1000

rx_phase_offset1 = 52.9170966 * np.pi / 180.0

d_wavelength = 0.0625 # Distance between elements in meters
radius = .0625
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
sdr.tx_enabled_channels = [0,1]
sdr.tx_rf_bandwidth = int(fc0*3)
sdr.tx_lo = int(tx_lo_low)
sdr.tx_cyclic_buffer = True
sdr.tx_hardwaregain_chan0 = int(tx_gain_0)
sdr.tx_hardwaregain_chan1 = int(tx_gain_1)
sdr.tx_buffer_size = int(2**18)

'''Program Tx and Send Data'''
fs = int(sdr.sample_rate)
N = 2**16
ts = 1 / float(fs)
t = np.arange(0, N * ts, ts)
i0 = np.cos(2 * np.pi * t * fc0) * 2 ** 14
q0 = np.sin(2 * np.pi * t * fc0) * 2 ** 14
i1 = np.cos(2 * np.pi * t * 1.5 * fc0) * 2 ** 14
q1 = np.sin(2 * np.pi * t * 1.5 * fc0) * 2 ** 14
iq0 = i0 + 1j * q0
iq1 = i1 + 1j * q1
sdr.tx([iq0,iq1])  # Send Tx data.

# Assign frequency bins and "zoom in" to the fc0 signal on those frequency bins
xf = np.fft.fftfreq(num_samples, ts)
xf = np.fft.fftshift(xf)/1e6
signal_start = int(num_samples*(samp_rate/2+fc0/2)/samp_rate)
signal_end = int(num_samples*(samp_rate/2+fc0*2)/samp_rate)

'''Configure arduino'''
arduino = ad.SerialDevice("COM5", 115200)
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

def plot3D(results):
    x = np.arange(results.shape[0])
    y = np.arange(results.shape[1])
    x,y = np.meshgrid(x,y)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(x,y,results.T)
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
    thetaScan = np.linspace(-1*np.pi/2, np.pi/2, resolution)
    X = getX()
    for theta in thetaScan:
        w = w_mvdr(theta, X)
        X_weight = w.H @ X
        power = 10*np.log10(np.var(X_weight))
        res.append(power)
    return np.array(res)


"""
phi measures deflection from the z vector
theta measures angle wittershins from x vector
"""
def spherical2cartesian(theta, phi, rho):
    x = rho*np.cos(theta)*np.sin(phi)
    y = rho * np.sin(theta)*np.sin(phi)
    z = rho * np.cos(phi)
    return np.array([x,y,z])

def cartesian2spherical(x,y,z):
    rho = np.sqrt(x**2 + y**2 + z**2)
    theta = np.arctan2(x, y)
    phi = np.arctan2(np.sqrt(x**2 + y**2), z)

"""Retruns the shortest distance between an point and a plane"""
def point2plane(point, normal):
    return np.abs(normal[0]*point[0] + normal[1]*point[1] + normal[2]*point[2]) / np.linalg.norm(normal)


"""
Conventions for this class
phi measures deflection from the z vector
theta measures angle wittershins from x vector
"""
def genArrayA(thetas:np.ndarray, phis:np.ndarray, rxPos:np.ndarray):
    if np.max(rxPos) > 3*np.pi:
        rxPos *= (np.pi/180.0)
    A = []
    for i, theta in enumerate(thetas):
        A_t = []
        for j, phi in enumerate(phis):
            a = np.zeros(rxPos.shape, dtype=np.complex128)
            normalCar = spherical2cartesian(theta, phi, 1)
            phaseShiftZero = 0
            for k, rx in enumerate(rxPos):
                # First calculate the distance from the impart plane to the rx
                if type(rx) is not np.ndarray:
                    rxCar = spherical2cartesian(rx, np.pi/2, wavelength/4)
                else:
                    rxCar = rx
                d2plane = point2plane(rxCar, normalCar)
                phaseShift = d2plane / 3e8 * rx_lo * np.pi * 2
                if (np.dot(rxCar, normalCar)) > 0:
                    phaseShift = -phaseShift
                if k == 0:
                    phaseShiftZero = phaseShift
                a[k] = np.exp(1j*(phaseShift - phaseShiftZero))

            A_t.append(a)
        A.append(np.array(A_t))
    return np.array(A)

def genArrayA2(thetas, phis, rxPos):
    if np.max(rxPos) > 3*np.pi:
        rxPos *= (np.pi/180.0)
    A = []
    for i, theta in enumerate(thetas):
        A_t = []
        for j, phi in enumerate(phis):
            a = np.zeros(rxPos.shape, dtype=np.complex128)
            for k, rx in enumerate(rxPos):
                a[k] = np.exp(2j*np.pi*(radius/wavelength) *np.sin(theta)*np.cos(phi-(2*np.pi*k/len(rxPos))))
            A_t.append(a)
        A.append(np.array(A_t))
    return np.array(A)



def rotorMUSIC(thetaRes=180, phiRes=45, rxAngle=[-45, 45]):
    arduino.reset()
    arduino.gotoBoundAngle(rxAngle[0])
    arduino.setSpeed(10)
    # get signal X from all rxAngles
    X = np.matrix(sdr.rx())
    times = [time.time_ns()]
    angles = [rxAngle[0], rxAngle[0]+180]
    for angle in rxAngle[1:]:
        arduino.gotoBoundAngle(angle)
        X = np.concatenate((X, sdr.rx()))
        times.append(time.time_ns)
        angles.append(angle)
        angles.append(angle+180)

    # Shift all rx by their time of arrival
    for t in times[1:]:
        pass
    # Generate all steering vectors

def spinAquire(angleResolution=1, angleMin=-90, angleMax=90):
    arduino.reset()
    arduino.setMode(ad.BOUNDED)
    arduino.gotoBoundAngle(angleMin)

    try:
        totalAngles = (angleMax - angleMin) // angleResolution
        angles = np.linspace(angleMin, angleMax, totalAngles)
        dfCaptures = np.zeros((2*totalAngles, num_samples), dtype=np.complex128)
        dfAngles = np.concatenate((angles, angles + 180))
        dfTimes = np.zeros(totalAngles, dtype=np.complex128)
        for i, angle in enumerate(angles):
            arduino.gotoBoundAngle(angle)
            dfTimes[i] = time.time_ns()
            rxData = sdr.rx()
            dfCaptures[i] += rxData[0]
            dfCaptures[i + totalAngles] += rxData[1] * np.exp(1j*rx_phase_offset1)
    except Exception as e:
        print(e)
    arduino.gotoBoundAngle(0)
    dfTimes = np.concatenate((dfTimes, dfTimes))
    dfTimes -= np.min(dfTimes)
    return dfCaptures, dfAngles, dfTimes


def spinDF(elementsPerGroup=4, dfResolution=90, angleResolution=1, angleMin=-90, angleMax=90, signals=2):
    dfCaptures, dfAngles, dfTimes = spinAquire(angleResolution, angleMin, angleMax)
    for i in range(dfCaptures.shape[0]):
        dfCaptures[i] *= np.exp(-2j*np.pi * rx_lo * (dfTimes[i]*10e-9))
    A = genArrayA2(np.linspace(0,np.pi/2, dfResolution),
                  np.linspace(0, 2*np.pi, 4*dfResolution),
                  dfAngles)

    dfCapturesStride = len(dfCaptures) // elementsPerGroup

    power = np.zeros(A.shape[:2])
    music = np.zeros(A.shape[:2])
    for k in range(dfCapturesStride):
        x = np.matrix(dfCaptures[[l*dfCapturesStride+k for l in range(elementsPerGroup)]])
        R = x@x.H
        if R.shape[0] != elementsPerGroup:
            print("need to swap x@x.H")
            return
        Rinv = np.linalg.pinv(R)
        w,v = np.linalg.eig(R)
        eig_order = np.argsort(np.abs(w))
        v = v[:, eig_order]
        V = np.matrix(np.zeros((elementsPerGroup, elementsPerGroup-signals), dtype=np.complex128))
        for i in range(elementsPerGroup-signals):
            V[:,i] = v[:,i]

        for i in range(A.shape[0]):
            for j in range(A.shape[1]):
                # make covariance matrix using elements k, k+90, k+180, k+ 270
                # make weight matrix from A
                s = np.matrix(A[i,j,[l*dfCapturesStride+k for l in range(elementsPerGroup)]]).T
                # add power
                power[i,j] += np.array(1/np.abs((s.H @ Rinv @ s))).squeeze()
                music[i,j] += np.array(1/np.abs(s.H @ V @ V.H @ s)).squeeze()

    return power, music
            



def rotorDF(dfResolution=90, angleResolution=1, angleMin=-90, angleMax=90):
    # Version 2 rotor df at each angle
    # init
    arduino.reset()
    arduino.setDirection(ad.CLOCKWISE)
    arduino.setMode(ad.BOUNDED)
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

if __name__ == "__main__":
    print("here")
    angles, dfResults = rotorDF()
    plotRect(angles, dfResults)
    np.save('df_1_-90_90(2).npy', dfResults)
    sdr.tx_destroy_buffer()