from scipy.odr import ODR, Model, Output, RealData
import scipy
import numpy as np

scipy.test()

x = np.array([1, 2, 3, 4, 5])
y = np.array([7, 6, 5, 4, 3])

def model(p, x) -> float:
    return p[0] * x * p[1]

model2 = Model(model)
data = RealData(x, y)
odr_model = ODR(data, model2, beta0=(0., 0.))
out = odr_model.run()
print(out.stopreason)
print(out.beta)
