# Knowledge about interview for control/robotics engineer

## IMU

### 6-axis IMU returns linear and angular accelerations but normally the angular raw data from the gyroscope is not actually accurate with drift -> filter out the pose by linear acceleration 

### gyro data should be complemetary to accel data (differentiate on accel should match with gyro or integrating gyro should match with the accel)

### complementary filter pipline: 1. Based on the last timestep orientation, the global frame with gravity could be estimated. 2. Use the normalized linear acceleration vector to calculate the error by performing corss product with the result from 1. 3. perform PI controller to get the corrected gyroscope data. 4. Combine the corrected gyro and last timestep orientation to update current orientation quaternion and normalize it 5. Use appropriate euler angle axis to get the orientation angles 

## Kalman filters (optimal luenburger observer)
### Its same as the Minimum Mean Square Error Estimator -> $K = P_{k|k-1}C'(CP_{K|K-1}C'+R)^{-1}$

## Embedded knowlegde
### Signal protocol
- SPI: 4 lines between master and slave. matser: cs+sclk+mosi+miso, slave: cs+sclk+sdaIn+sdaOu. 1-100Mhz full-duplex
- CAN: Tx and Rx, Can ID, Can high and Can low has a 120 ohm resistor
- I2C: two lines , slower data transmision rate than spi, half-duplex
- UART