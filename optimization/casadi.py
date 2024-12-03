import ipyopt.optimize
import numpy as np 
# import casadi as ca
from scipy.optimize import minimize, Bounds
from scipy.optimize._numdiff import approx_derivative
from cyipopt import minimize_ipopt
import time
import matplotlib.pyplot as plt

'''
    state = [x, x_dot, x_ddot, u]
    x is R^6
'''
def object_func(x, K):
    tf = x[0]
    delta_t = tf/(K-1)
    error = 1/tf
    x = x[1:]
    for i in range(K-1):
        error += np.sum(x[i*(6*4)+12:(i+1)*(6*4)]**2) * delta_t
    return error
    
def model_constraint(x):
    tf = x[0]
    x = x[1:]
    K = int(len(x) / 24)
    delta_t = tf / (K-1)
    # print(f'K: {K}')
    A = np.zeros((18,18))
    A[:6,6:12] = np.identity(6)
    A[6:12,12:18] = np.identity(6)
    B = np.zeros((18,6))
    B[12:, :] = np.identity(6)
    residue = np.zeros((18,K-1))
    for i in range(K-1):
        state_x = x[i*(6*4):i*(6*4)+18].reshape(-1,1)
        u = x[i*(6*4)+18:i*(6*4)+24].reshape(-1,1)
        state_xdot = x[(i+1)*(6*4):(i+1)*(6*4)+18].reshape(-1,1)
        # print(f'shape: x: {state_x.shape}, u: {u.shape}, x_dot: {state_xdot.shape}')
        # residue += state_xdot - (A@state_x + B@u)
        residue[:,i] = (state_xdot - ((A@state_x+B@u)*delta_t + state_x)).reshape(18)
    ret = residue.flatten()
    # print(f'ret shape: {ret.shape}')
    return ret
        
def main():
    tf = 0.45
    K = 30
    delta_t = tf/(K-1)
    target = np.array([0, -0.5, 0.45, np.pi/2, 0, 0])
    x0 = 0.1*np.ones(K*24+1)
    print(f'x0 shape: {x0.shape}')
    solver = 'SLSQP' # Can also try ipopt (内点法) 
    cons = [{'type':'eq', 'fun':lambda x: model_constraint(x), 'jac': lambda x: approx_derivative(lambda xx: model_constraint(xx), x)}]
    lb = np.concatenate((np.array([0]), -1*np.ones(6), np.zeros(18)), axis=0)
    ub = np.concatenate((np.array([tf]), 1*np.ones(6), np.zeros(18)), axis=0)
    for i in range(K-2):
        lb = np.concatenate((lb, -np.inf*np.ones(6), np.zeros(6), -np.inf*np.ones(6), -np.inf*np.ones(6)), axis=0)
        ub = np.concatenate((ub, np.inf*np.ones(6), np.inf*np.ones(6), np.inf*np.ones(6), np.inf*np.ones(6)), axis=0)
    lb = np.concatenate((lb, target, -np.inf*np.ones(18)), axis=0)
    ub = np.concatenate((ub, target, np.inf*np.ones(18)), axis=0)
    # lb = np.array([-0.5*np.ones(6), np.zeros(18), -np.inf*np.ones((K-2)*24),target, -np.inf*np.ones(18)])
    # ub = np.array([0.5*np.ones(6), np.zeros(18), np.inf*np.ones((K-2)*24),target, np.inf*np.ones(18)])
    bnds = Bounds(lb=lb, ub=ub)
    start = time.time()
    fun = lambda x: object_func(x, K=K)
    # res = minimize(fun=fun, x0=x0, method=solver,
    #                bounds=bnds, constraints=cons)
    execution_time = time.time() - start
    # print(f'solution: {res.x} with the error {res.fun}\n time: {execution_time}')
    print(f'execution time: {execution_time}')
    
    # with ipopt method
    res = minimize_ipopt(fun=fun, x0=x0, bounds=bnds, constraints=cons, options={'disp': 5})
    ans = res.x[1:]
    new_tf = res.x[0]
    print(f"optimized t: {new_tf}")
    new_delta_t = new_tf/(K-1)
    
    delta_k = 10
    def get_values(offset):
        x = []
        t = []
        for i in range(K-1):
            cur_x = ans[i*24+offset]
            next_x = ans[(i+1)*24+offset]
            delta_x = next_x - cur_x
            cur_t = i*new_delta_t
            for j in range(delta_k):
                t.append(cur_t+new_delta_t/delta_k*j)
                x.append(cur_x+delta_x/delta_k*j)
        return x, t
    x, t = get_values(0)
    y, _ = get_values(1)
    z, _ = get_values(2)
    vx, _ = get_values(6)
    vy, _ = get_values(8)
    vz, _ = get_values(8)
    ax, _ = get_values(12)
    ay, _ = get_values(13)
    az, _ = get_values(14)
    jx, _ = get_values(18)
    jy, _ = get_values(19)
    jz, _ = get_values(20)
    

    fig = plt.figure()
    axes = fig.add_subplot(2,2,1)
    axes.plot(t, x, 'g', label='x')
    axes.plot(t, y, 'r', label='y')
    axes.plot(t, z, 'b', label='z')
    axes = fig.add_subplot(2,2,2)
    axes.plot(t, vx, 'g', label='vx')
    axes.plot(t, vy, 'r', label='vy')
    axes.plot(t, vz, 'b', label='vz')
    axes = fig.add_subplot(2,2,2)
    axes.plot(t, vx, 'g', label='vx')
    axes.plot(t, vy, 'r', label='vy')
    axes.plot(t, vz, 'b', label='vz')
    axes = fig.add_subplot(2,2,3)
    axes.plot(t, ax, 'g', label='ax')
    axes.plot(t, ay, 'r', label='ay')
    axes.plot(t, az, 'b', label='az')
    axes = fig.add_subplot(2,2,4)
    axes.plot(t, jx, 'g', label='ax')
    axes.plot(t, jy, 'r', label='ay')
    axes.plot(t, jz, 'b', label='az')


    
    plt.show()

if __name__ == "__main__":
    # A = np.zeros((18,18))
    # A[:6,6:12] = np.ones((6,6))
    # A[6:12,12:18] = np.ones((6,6))
    # print(f'A: {A}, shape: {A.shape}')
    # target = np.array([0, -0.5, 0.45])
    # K=8
    # lb = np.concatenate((-0.5*np.ones(6), np.zeros(18), -np.inf*np.ones((K-2)*24),target, -np.inf*np.ones(18)), axis=0)
    # print(f'lb: {lb}, shape: {lb.shape}')
    main()