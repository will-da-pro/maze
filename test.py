import numpy as np
import scipy.odr as odrpack
np.random.seed(1)

N = 100
x = np.linspace(0,10,N)
y = 3*x - 1 + np.random.random(N)
sx = np.random.random(N)
sy = np.random.random(N)

def f(B, x):
    return B[0]*x + B[1]
linear = odrpack.Model(f)
# mydata = odrpack.Data(x, y, wd=1./np.power(sx,2), we=1./np.power(sy,2))
mydata = odrpack.RealData(x, y, sx=sx, sy=sy)

myodr = odrpack.ODR(mydata, linear, beta0=[1., 2.])
myoutput = myodr.run()
myoutput.pprint()
