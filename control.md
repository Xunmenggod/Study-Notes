# Knowledge about interview for control/robotics engineer

## Basics (math)
### Orientation
- Quternion representation (w, x, y, z), conjugate operation -q = (w, -x, -y, -z), quaternion product可以表示两个旋转的叠加， quaternion inverse $q^{-1} = \frac{q^*}{|q|}=q^*$(if it is a unit quaternion); Quternion error $= q1*q2^*$; Qunternion error to angular velocity error: 
  1. convert quaternion error to axis angle: angle = 2 * arccos(abs(w)), axis = (x,y,z) / sin(angle)
  2. calculate the angular displacement: dw = axis * angle
  3. differentiate to get the  velocity_error = dw / dt

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


## Optimal control
- To tracking a reference instead of the regulating problem, we used to set the delta u as the optimized variable instead of directly optimizing the control problem. The reason is because when error is converged to zero, the control variable will be zero. And we could not set the control variable to be zero for tracking situation,then it cannot achieve zero error steady-state. So, we need to shift to optimize $\Delta{u}$ to aware the previous problem during optimization.

- In practise for MPC problem, we introduced a $N_u$ as the number of control horizon which is much smaller than the prediction horizon, N (the horizon to optimize the cost). In the time steps that are larger than control horizon, but smaller than the prediction horizon, the delta control variable is set to zero. Normally $N_u$ is set to 2 or 3.


## Embedded knowlegde
### Signal protocol
- SPI: 4 lines between master and slave. matser: cs+sclk+mosi+miso, slave: cs+sclk+sdaIn+sdaOu. 1-100Mhz full-duplex
- CAN: Tx and Rx, Can ID, Can high and Can low has a 120 ohm resistor
- I2C: two lines , slower data transmision rate than spi, half-duplex
- UART