from sklearn.pipeline import make_pipeline
from sklearn.svm import SVC
import numpy as np
import sklearn


class ArrClass:
    def __init__(self):
        self.numAntenna = 3
        self.samps = 100
        self.fs = 100
        self.f = 20
    
def genArrayVec(numAntenna:int, theta:float) -> np.ndarray: 
    return np.array([np.exp(-2j*np.pi*.5*np.sin(theta) * i) for i in range(numAntenna)])

def genData(numAntenna:int, samps:int, fs:float, f:float, theta:float) -> np.ndarray:
    a = genArrayVec(numAntenna=numAntenna, theta=theta)
    t = np.arange(samps)/fs + np.random.randn(1)
    tx = np.exp(2j * np.pi * t * f)
    a = a.reshape(-1, 1)
    tx = tx.reshape(-1, 1)
    return a @ tx.T

def genSweep(arrClass:ArrClass, dimm, start:int=0, end:int=90, step:int=1):
    y = np.array([])
    x = np.array([])
    for i in np.arange(start, end, step):
        ys = np.zeros(arrClass.samps) + i
        xs = genData(arrClass.numAntenna, arrClass.samps, arrClass.fs, arrClass.f, i)
        y = np.concatenate(y, ys)

ac = ArrClass()

headon = genData(numAntenna=ac.numAntenna, samps=200, fs=100, f=20, theta=0)
quarter = genData(numAntenna=ac.numAntenna, samps=200, fs=100, f=20, theta=45)

xHead = headon.T.reshape(-1, 6)
xquarter = quarter.T.reshape(-1, 6)
x = np.concatenate((xHead, xquarter))
x = np.concatenate((x.real, x.imag), 1)
y = np.concatenate((np.zeros(100), np.ones(100)))

test_headon = genData(numAntenna=ac.numAntenna, samps=200, fs=100, f=20, theta=0)
test_quarter = genData(numAntenna=ac.numAntenna, samps=200, fs=100, f=20, theta=45)

test_xHead = test_headon.T.reshape(-1, 6)
test_xquarter = test_quarter.T.reshape(-1, 6)
test_x = np.concatenate((test_xHead, test_xquarter))
test_x = np.concatenate((test_x.real, test_x.imag), 1)
test_y = np.concatenate((np.zeros(100), np.ones(100)))

clf = sklearn.pipeline.make_pipeline(sklearn.preprocessing.StandardScaler(), sklearn.svm.SVC(gamma='auto', kernel='rbf'))
# svm_modle = SVC(gamma='auto')
# svm_modle.fit(x, y)
clf.fit(x, y)
print(clf)
print(clf.predict([test_x[150]]))
print(sklearn.metrics.accuracy_score(test_y, clf.predict(test_x)))