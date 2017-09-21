/* This file is part of the Razor AHRS Firmware */

void Compass_Heading()
{
  float mag_x;
  float mag_y;
  float mag_z;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  float variation;

  cos_roll = cos(average_roll);
  sin_roll = sin(average_roll);
  cos_pitch = cos(average_pitch);
  sin_pitch = sin(average_pitch);
  variation = 16.5 * (PI / 180); //inclination for San Diego, CA - Change to your location to get True North

  //The magnetic heading was noisy so I used a smoothing filter to smooth reduce the noise
  total_MAG_Heading = total_MAG_Heading - MAG_Heading_readings[readIndex];

  // Tilt compensated magnetic field X
  mag_x = magnetom[0] * cos_pitch + magnetom[1] * sin_roll * sin_pitch + magnetom[2] * cos_roll * sin_pitch;
  // Tilt compensated magnetic field Y
  mag_y = magnetom[1] * cos_roll - magnetom[2] * sin_roll;
  MAG_Heading = atan2(-mag_y, mag_x);
  MAG_Heading += variation;
  
  if (MAG_Heading > PI) {
	  MAG_Heading -= (2 * PI);
  }
  if (MAG_Heading < -PI) {
	  MAG_Heading += (2 * PI);
  }
  if (MAG_Heading < 0) {
	  MAG_Heading += 2 * PI;
  }
  MAG_Heading_readings[readIndex] = MAG_Heading;
  total_MAG_Heading = total_MAG_Heading + MAG_Heading_readings[readIndex];

  readIndex = readIndex + 1;

  if (readIndex >= numReadings) {
	  readIndex = 0;
  }
  average_MAG_Heading = total_MAG_Heading / numReadings;
}
