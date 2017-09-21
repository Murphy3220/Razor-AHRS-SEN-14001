/* This file is part of the Razor AHRS Firmware */

/*Several of the axis had to be flipped to make the AHRS system work for the SEN-14001 */

// Reads x, y and z accelerometer registers
void Read_Accel()
{
    // We want the gravity vector, which is negated acceleration vector.
	accel[0] = -imu.ax;  // X axis (accelerometer -x axis)
    accel[1] = imu.ay;  // Y axis (accelerometer sensor y axis)
    accel[2] = imu.az;  // Z axis (accelerometer sensor z axis)
}

void Read_Magn()
{
	magnetom[0] = imu.my;   // X axis (magnetic sensor y axis)
	magnetom[1] = -imu.mx;  // Y axis (magnetic sensor -x axis)
	magnetom[2] = imu.mz; // Z axis (magnetic sensor z axis)
}

// Reads x, y and z gyroscope registers
void Read_Gyro()
{
	gyro[0] = imu.gx;    // X axis (gyro sensor x axis)
    gyro[1] = -imu.gy;    // Y axis (gyro sensor -y axis)
    gyro[2] = -imu.gz;    // Z axis (gyro sensor -z axis)
}
