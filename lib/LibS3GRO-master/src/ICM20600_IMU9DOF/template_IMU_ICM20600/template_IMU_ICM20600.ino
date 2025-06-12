#include "IMU9DoF_ICM20600.h"
#include "I2Cdev.h"

// Create an instance of the IMU9DOF class
IMU9DOF imu(true);

void setup() {
    // Start serial communication
    Serial.begin(115200);  
    Serial.println("Initializing IMU9DOF...");
    delay(1000);

    Wire.begin();
    Serial.println("I2C communication initialized.");
    
    // Initialize the IMU
    imu.init();

    Serial.println("Initialization complete.");
    
}

void loop() {
    Serial.println();
    // Check if the sensor is connected
    if (imu.isConnected()) {
        // Read acceleration data
        double accelX = imu.getAccelX();
        double accelY = imu.getAccelY();
        double accelZ = imu.getAccelZ();
        
        // Read gyroscope data
        double gyroX = imu.getGyroX();
        double gyroY = imu.getGyroY();
        double gyroZ = imu.getGyroZ();
        
        // Read temperature
        double temp = imu.getTemp();
        
        // Print the data to the Serial Monitor
        Serial.println("Accel X: "); Serial.print(accelX); Serial.print(" g");
        Serial.println("\tAccel Y: "); Serial.print(accelY); Serial.print(" g");
        Serial.println("\tAccel Z: "); Serial.print(accelZ); Serial.print(" g");
        Serial.println("\tGyro X: "); Serial.print(gyroX); Serial.print(" °/s");
        Serial.println("\tGyro Y: "); Serial.print(gyroY); Serial.print(" °/s");
        Serial.println("\tGyro Z: "); Serial.print(gyroZ); Serial.print(" °/s");
        Serial.println("\tTemp: "); Serial.print(temp); Serial.print(" °C");
        Serial.println();
    } else {
        Serial.println("IMU not connected.");
    }
    
    // Delay before next read
    delay(1000);
}
