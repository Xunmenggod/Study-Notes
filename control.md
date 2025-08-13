# Knowledge about interview for control/robotics engineer

## Basics (math)
### Orientation
- Quternion representation (w, x, y, z), conjugate operation -q = (w, -x, -y, -z), quaternion product可以表示两个旋转的叠加， quaternion inverse $q^{-1} = \frac{q^*}{|q|}=q^*$(if it is a unit quaternion); Quternion error $= q1*q2^*$; Qunternion error to angular velocity error: 
  1. convert quaternion error to axis angle: angle = 2 * arccos(abs(w)), axis = (x,y,z) / sin(angle)
  2. calculate the angular displacement: dw = axis * angle
  3. differentiate to get the  velocity_error = dw / dt

- Pose (SE(3)) differntaition -> Twist (se(3))
- To get the ==error between two SE(3)==: $err = log(T_{des}^{-1}T_{cur})$
- Linear is normal differentaition over time
- Angular: 1. err_quat = cur_quat * last_quat_conjugate 2. axis = xyz(err_quat) 3. norm = ||axis||_2, axis = axis/norm 4. speed = 2 * atan2(norm, w(err_quat)) 5. speed = speed /.dt 6. angular_vel = speed * axis
```python
  def convert_ori_to_vel(last_ori, cur_ori, dt):
    err_quat = cur_ori * quat_conjugate(last_ori)
    
    axis = err_quat.xyz()
    norm = axis.norm()
    if norm < 1e-15:
      axis = [1,0,0]
    else:
      axis = 1/norm * axis
    
    speed = 2 * atan2(norm, err_quat.w())
    if speed > pi:
      speed -= 2*pi
    speed /= dt

    angular_vel = speed * axis
    
```

### Homogenous tranformation:
- $T^{-1} = [R^T, -R^Tb; 0, 1]$
- To rotate a point in b to s frame by $p_s = T_{sb}p_b$
- T_sb has the rotation matrix from s to b, has the trans vector that is the origin of b in s frame

### Transformation for Twist & Wrench
- $Ad_T = [R 0; \hat{p}R, R]$
- $\hat{p}$ is skew-symmerty matrix of the vector p, $\hat{p} = [0, -z, y; z, 0, -x; -y, x, 0]$
- $V_a = Ad_{T_{ab}}V_b$
- $F_a = [Ad_{T_{ba}}]^TF_b$

### Jacobian
- Given a velocity jacobian in a specified frame A, how could we transfer the jacobian in a different frame B
  If those two frames have same orientation, then the jacobian in different frame is same, otherwise the jacobian in frame B will equal to
  BlkDiag(rot_mtx_B2A, rot_mtx_B2A) * jacobian in A
- **Notice**: The angular velocity in different frames but the frames have same orientation will have same value

### hybrid force-position controller
- Robotic tasks division: in contact environment(constrained direction) or in free space
- For constrained space: position and force controlled "direction"
  There are several variables that we could not controlled such as normal direction position is limited, tangential force is limited and governed by the friction cone
  For controlled direction: position in tangential direction and the normal force to the surface exerted by the robot
  They configure the two complementary set of variables for constrined direction
- Impedance control
- Joint space: $\tau = M(q)\ddot{q}_d + h(q,\dot{q}) + M(q)M_d^{-1}(D_d\dot{\tilde{q}} + K_d\tilde{q}) + (I-M(q)M_d^{-1})\tau_{ext}$ (Ideal impedance control); $\tau =  M(q)\ddot{q}_d + h(q,\dot{q}) + M(q)M_d^{-1}(D_d\dot{\tilde{q}} + K_d\tilde{q})$ (Weak coupling impedance control);  $\tau = h(q,\dot{q}) -D_d\dot{q} + K_d\tilde{q}$ (PD+control/Joint compliant control)
- Cartesian space: $\tau = M(q)J(q)^{-1}M_d^{-1}(M_d\ddot{x} + D\dot{\tilde{x}} + K\tilde{x} - M_d\dot{J}(\dot{q},q)\dot{q}) + 
  (J^T(q) - M(q)J^{-1}(q)M_d^{-1})F_ext + h(q,\dot{q})$ (Ideal impedance control); $\tau = M(q)J(q)^{-1}M_d^{-1}(M_d\ddot{x} + D\dot{\tilde{x}} + K\tilde{x} - M_d\dot{J}(\dot{q},q)\dot{q}) + h(q,\dot{q})$ (Without end effector feedback); $\tau = J^T(q)(D\dot{\tilde{x}} + K\tilde{x}) + h(q, \dot{q})$ (Little impedance characteristics but with larger position tracking error)


## IMU

- 6-axis IMU returns linear and angular accelerations but normally the angular raw data from the gyroscope is not actually accurate with drift -> filter out the pose by linear acceleration 

- gyro data should be complemetary to accel data (differentiate on accel should match with gyro or integrating gyro should match with the accel)

### complementary filter pipline: 
1. Based on the last timestep orientation, the global frame with gravity could be estimated. 
2. Use the normalized linear acceleration vector to calculate the error by performing corss product with the result from 1. 
3. perform PI controller to get the corrected gyroscope data. 
4. Combine the corrected gyro and last timestep orientation to update current orientation quaternion and normalize it 
5. Use appropriate euler angle axis to get the orientation angles 

## Kalman filters (optimal luenburger observer)
### Its same as the Minimum Mean Square Error Estimator -> $K = P_{k|k-1}C'(CP_{K|K-1}C'+R)^{-1}$


## Optimization
### ADMM
- optimization problem: $\mathop{\arg\min}\limits_{x,z} f(x) + g(z) \qquad s.t. Ax+Bz=c$
- f & g must be convex, equality constraint is the affine transformation
- Augmented Lagrandian: $L(x,z,y) = f(x) + g(z) + y^T(Ax+Bz-c) + (\frac{\rho}{2})||Ax+Bz-c||^2_2$, the first three terms are normal Lagrangian, and the last term is to enforce the equality constraint h(x,z)
- Algorithm
- 1. $x^{k+1} = \mathop{\arg\min}\limits_{x} L(x,z^k,y^k)$
- 2. $z^{k+1} = \mathop{\arg\min}\limits_{z} L(x^{k+1},z,y^k)$
- 3. $y^{k+1} = y^k + \rho(h(x^{k+1},z^{k+1}))$
- trick: we could update $\rho$ as $\frac{1}{k}$, k is the iteration number of the algorithm


## Optimal control
- To tracking a reference instead of the regulating problem, we used to set the delta u as the optimized variable instead of directly optimizing the control problem. The reason is because when error is converged to zero, the control variable will be zero. And we could not set the control variable to be zero for tracking situation,then it cannot achieve zero error steady-state. So, we need to shift to optimize $\Delta{u}$ to aware the previous problem during optimization.

- In practise for MPC problem, we introduced a $N_u$ as the number of control horizon which is much smaller than the prediction horizon, N (the horizon to optimize the cost). In the time steps that are larger than control horizon, but smaller than the prediction horizon, the delta control variable is set to zero. Normally $N_u$ is set to 2 or 3.

### iterative LQR for nonlinear system dynamics
- 

## Embedded knowlegde
### Signal protocol
- SPI: 4 lines between master and slave. matser: cs+sclk+mosi+miso, slave: cs+sclk+sdaIn+sdaOu. 1-100Mhz full-duplex
- CAN: Tx and Rx, Can ID, Can high and Can low has a 120 ohm resistor
- I2C: two lines , slower data transmision rate than spi, half-duplex
- UART