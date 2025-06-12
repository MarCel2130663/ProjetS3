/*
Projet S3 GRO 2024
Class to communicate with ICM20600
@author Frédéric Therrien
@version 1.0 24/07/24
*/
#ifndef IMU9DOF_ICM20600_h
#define IMU9DOF_ICM20600_h

#include "ICM20600.h"
#include "I2Cdev.h" 

class IMU9DOF20600
{
  public:
    /** Constructor to initialize the IMU with AD0 state (Needs to be set to true) */
    IMU9DOF20600(bool ad0_state);

    /** Method to initialize communication between Arduino and IMU9DOF */
    void init();

    /** Method to set the power mode of the IMU */
    void setPowerMode(icm20600_power_type_t mode);

    /** Method to set the accelerometer scale range */
    void setAccScaleRange(acc_scale_type_t range);

    /** Method to set the gyroscope scale range */
    void setGyroScaleRange(gyro_scale_type_t range);

    /** Method to poll the availability of the sensor
    @return true if available */
    bool isConnected() { return imu_.getDeviceID() != 0; };

    /** Method to get acceleration data in X
    @return double value in g [-2, 2] */
    double getAccelX();

    /** Method to get acceleration data in Y
    @return double value in g [-2, 2] */
    double getAccelY();

    /** Method to get acceleration data in Z
    @return double value in g [-2, 2] */
    double getAccelZ();

    /** Method to get gyroscope data in X
    @return double value in degrees/s [-250, 250] */
    double getGyroX();

    /** Method to get gyroscope data in Y
    @return double value in degrees/s [-250, 250] */
    double getGyroY();

    /** Method to get gyroscope data in Z
    @return double value in degrees/s [-250, 250] */
    double getGyroZ();

    /** Method to get temperature
    @return double value in degrees C */
    double getTemp();

  private:
    ICM20600 imu_; 
};

#endif // IMU9DOF_ICM20600_h