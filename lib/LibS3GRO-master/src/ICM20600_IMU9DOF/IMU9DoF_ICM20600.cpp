#include "IMU9DoF_ICM20600.h"

IMU9DOF20600::IMU9DOF20600(bool ad0_state) : imu_(ad0_state) {

}

void IMU9DOF20600::init() {
    ;

    // Initialization du IMC20600

    imu_.initialize();
    

    // Vérification de l'initialization
    uint8_t deviceID = imu_.getDeviceID();
    
    if (deviceID == ICM20600_I2C_ADDR1 || deviceID == ICM20600_I2C_ADDR2) {
        Serial.println("ICM20600 sensor initialized successfully.");
    } else {
        Serial.println("ICM20600 sensor initialization failed.");
    }

    // Paramètres optionnel
    imu_.setPowerMode(ICM_6AXIS_LOW_NOISE);
    imu_.setAccScaleRange(RANGE_2G);
    imu_.setGyroScaleRange(RANGE_250_DPS);
}


void IMU9DOF20600::setPowerMode(icm20600_power_type_t mode) {
    imu_.setPowerMode(mode);
}
// Définition du range d'accel
void IMU9DOF20600::setAccScaleRange(acc_scale_type_t range) {
    imu_.setAccScaleRange(range);
}
// Définition du range du gyroscope
void IMU9DOF20600::setGyroScaleRange(gyro_scale_type_t range) {
    imu_.setGyroScaleRange(range);
}
// fonction pour avoir l'accel en X
double IMU9DOF20600::getAccelX() {
    int16_t x, y, z;
    imu_.getAcceleration(&x, &y, &z);
    // Assuming full scale range of ±2g, convert raw data to g
    return x / 16384.0; // 16384 is the scaling factor for ±2g range
}
// fonction pour avoir l'accel en Y
double IMU9DOF20600::getAccelY() {
    int16_t x, y, z;
    imu_.getAcceleration(&x, &y, &z);
    return y / 16384.0;
}
// fonction pour avoir l'accel en Z
double IMU9DOF20600::getAccelZ() {
    int16_t x, y, z;
    imu_.getAcceleration(&x, &y, &z);
    return z / 16384.0;
}
// fonction pour avoir le gyro en X
double IMU9DOF20600::getGyroX() {
    int16_t x, y, z;
    imu_.getGyroscope(&x, &y, &z);
    // Assuming full scale range of ±250 degrees/s, convert raw data to degrees/s
    return x / 131.0; // 131 is the scaling factor for ±250 degrees/s range
}
// fonction pour avoir le gyro en Y
double IMU9DOF20600::getGyroY() {
    int16_t x, y, z;
    imu_.getGyroscope(&x, &y, &z);
    return y / 131.0;
}
// fonction pour avoir le gyro en Z
double IMU9DOF20600::getGyroZ() {
    int16_t x, y, z;
    imu_.getGyroscope(&x, &y, &z);
    return z / 131.0;
}

// TODO: Implémenter cette fonction
double IMU9DOF20600::getTemp() {
    return -1.;
}