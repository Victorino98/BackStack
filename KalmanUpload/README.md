# BackStack

Needs the following libraries:
- Arduino_LSM6DS3
- Kalman (the version by TKJElectronics)

Other notes:
- Current configured to collect at 104Hz for both acceleration and gyro - not high enough
- Weird spiking problem that might be due to the restricted range to +/- pi - investigate
- calibration?
- power management testing


KalmanUpload:
 - edited file from Kalman_sherry
 - Includes upload to firebase
 - Uploads: kalmanX, kalmanY, hour, minute, second. 
