import numpy as np
import matplotlib.pyplot as plt
from cyipopt import minimize_ipopt
from scipy.optimize._numdiff import approx_derivative
import matplotlib.pyplot as plt

# z = (x1(t0) .... x1(tN) x2(t0) .... x2(tN) v(t0) .... v(tN))^T

def objective(z, time):
    x0, x1, v = np.split(z, 3)
    res = 0.0
    for i in range(time.size-1):
        h = time[i+1] - time[i]
        res += h*((x0[i]-1)**2 + (x1[i]-1)**2)
    return res


def ode_rhs(t, x, v):
    x0, x1 = x
    xdot1 = x0 - x0*x1 - 0.4*x0*v
    xdot2 = -x1 + x0*x1 - 0.2*x1*v
    return np.array([xdot1, xdot2])


def constraint(z, time):
    x0, x1, v = np.split(z, 3)
    x = np.array([x0, x1])
    res = np.zeros((2, x0.size))

    # initial values
    res[:, 0] = x[:, 0] - np.array([0.5, 0.7])

    # 'solve' the ode-system
    for j in range(time.size-1):
        h = time[j+1] - time[j]
        # implicite euler scheme
        res[:, j+1] = x[:, j+1] - x[:, j] - h*ode_rhs(time[j+1], x[:, j], v[j])
    # print(f'shape: {res.flatten().shape}, size: {x0.size}')
    return res.flatten()


# time grid
tspan = [0, 12]
dt    = 0.1
time  = np.arange(tspan[0], tspan[1] + dt, dt)

# initial point
z0 = 0.1 + np.zeros(time.size*3)

# variable bounds
bnds = [(None, None) if i < 2*time.size else (0, 1) for i in range(z0.size)]
# print(bnds)

# constraints:
cons = [{
    'type': 'eq', 
    'fun': lambda z: constraint(z, time), 
    'jac': lambda z: approx_derivative(lambda zz: constraint(zz, time), z)
}]

# call the solver
res = minimize_ipopt(lambda z: objective(z, time), x0=z0, bounds=bnds, 
                     constraints=cons, options = {'disp': 5})
x = res.x[:time.size]
plt.plot(time, x)
plt.show()
