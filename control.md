# Knowledge about interview for control/robotics engineer

## IMU

### 6-axis IMU returns linear and angular accelerations but normally the angular raw data from the gyroscope is not actually accurate with drift -> filter out the pose by linear acceleration 

### gyro data should be complemetary to accel data (differentiate on accel should match with gyro or integrating gyro should match with the accel)

### complementary filter pipline: 1. Based on the last timestep orientation, the global frame with gravity could be estimated. 2. Use the normalized linear acceleration vector to calculate the error by performing corss product with the result from 1. 3. perform PI controller to get the corrected gyroscope data. 4. Combine the corrected gyro and last timestep orientation to update current orientation quaternion and normalize it 5. Use appropriate euler angle axis to get the orientation angles 

## Kalman filters (optimal luenburger observer)
