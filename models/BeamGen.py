import numpy as np

def genArrayVec(numAntenna:int, theta:float) -> np.ndarray: 
    return np.array([np.exp(-2j*np.pi*.5*np.sin(theta) * i) for i in range(numAntenna)])

def genData(numAntenna:int, samps:int, fs:float, f:float, theta:float, noise:float=0) -> np.ndarray:
    a = genArrayVec(numAntenna=numAntenna, theta=theta)
    t = np.arange(samps)/fs
    n = np.random.randn(numAntenna, samps) + 1j * np.random.randn(numAntenna, samps)
    n = n * noise
    tx = np.exp(2j * np.pi * t * f)
    a = a.reshape(-1, 1)
    tx = tx.reshape(-1, 1)
    return (a @ tx.T) + n
