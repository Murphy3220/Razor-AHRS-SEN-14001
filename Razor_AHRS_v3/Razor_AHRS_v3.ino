/****************************************************
  "9DOF Razor IMU" hardware versions: SEN-14001

/*
  Axis definition (differs from definition printed on the board!):
    X axis pointing forward (towards the short edge with the connector holes)
    Y axis pointing to the right
    and Z axis pointing down.
    
  Positive yaw   : clockwise
  Positive roll  : right wing down
  Positive pitch : nose up
  
  Transformation order: first yaw then pitch then roll.
*/

/*
  Serial commands that the firmware understands:
  
  "#o<params>" - Set OUTPUT mode and parameters. The available options are:
  
      // Streaming output
      "#o0" - DISABLE continuous streaming output. Also see #f below.
      "#o1" - ENABLE continuous streaming output.
      
      // Angles output
      "#ob" - Output angles in BINARY format (yaw/pitch/roll as binary float, so one output frame
              is 3x4 = 12 bytes long).
      "#ot" - Output angles in TEXT format (Output frames have form like "#YPR=-142.28,-5.38,33.52",
              followed by carriage return and line feed [\r\n]).
      
      // Sensor calibration
      "#oc" - Go to CALIBRATION output mode.
      "#on" - When in calibration mode, go on to calibrate NEXT sensor.
      
      // Sensor data output
      "#osct" - Output CALIBRATED SENSOR data of all 9 axes in TEXT format.
                One frame consist of three lines - one for each sensor: acc, mag, gyr.
      "#osrt" - Output RAW SENSOR data of all 9 axes in TEXT format.
                One frame consist of three lines - one for each sensor: acc, mag, gyr.
      "#osbt" - Output BOTH raw and calibrated SENSOR data of all 9 axes in TEXT format.
                One frame consist of six lines - like #osrt and #osct combined (first RAW, then CALIBRATED).
                NOTE: This is a lot of number-to-text conversion work for the little 8MHz chip on the Razor boards.
                In fact it's too much and an output frame rate of 50Hz can not be maintained. #osbb.
      "#oscb" - Output CALIBRATED SENSOR data of all 9 axes in BINARY format.
                One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
      "#osrb" - Output RAW SENSOR data of all 9 axes in BINARY format.
                One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
      "#osbb" - Output BOTH raw and calibrated SENSOR data of all 9 axes in BINARY format.
                One frame consist of 2x36 = 72 bytes - like #osrb and #oscb combined (first RAW, then CALIBRATED).
      
      // Error message output        
      "#oe0" - Disable ERROR message output.
      "#oe1" - Enable ERROR message output.
    
    
  "#f" - Request one output frame - useful when continuous output is disabled and updates are
         required in larger intervals only. Though #f only requests one reply, replies are still
         bound to the internal 20ms (50Hz) time raster. So worst case delay that #f can add is 19.99ms.
         
         
  "#s<xy>" - Request synch token - useful to find out where the frame boundaries are in a continuous
         binary stream or to see if tracker is present and answering. The tracker will send
         "#SYNCH<xy>\r\n" in response (so it's possible to read using a readLine() function).
         x and y are two mandatory but arbitrary bytes that can be used to find out which request
         the answer belongs to.
          
          
  ("#C" and "#D" - Reserved for communication with optional Bluetooth module.)
  
  Newline characters are not required. So you could send "#ob#o1#s", which
  would set binary output mode, enable continuous streaming output and request
  a synch token all at once.
  
  The status LED will be on if streaming output is enabled and off otherwise.
  
  Byte order of binary output is little-endian: least significant byte comes first.
*/



/*****************************************************************/
/*********** USER SETUP AREA! Set your options here! *************/
/*****************************************************************/
#include <SparkFunMPU9250-DMP_2.h>

//#define SerialPort Serial1  //Use onboard RX/TX pins
#define SerialPort SerialUSB //Use USB for RX/TX

MPU9250_DMP imu;

// OUTPUT OPTIONS
/*****************************************************************/
// Set your serial port baud rate used to send out data here!
#define OUTPUT__BAUD_RATE 115200

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT__DATA_INTERVAL 20  // in milliseconds

// Output mode definitions (do not change)
#define OUTPUT__MODE_CALIBRATE_SENSORS 0 // Outputs sensor min/max values as text for manual calibration
#define OUTPUT__MODE_ANGLES 1 // Outputs yaw/pitch/roll in degrees
#define OUTPUT__MODE_SENSORS_CALIB 2 // Outputs calibrated sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_RAW 3 // Outputs raw (uncalibrated) sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_BOTH 4 // Outputs calibrated AND raw sensor values for all 9 axes
// Output format definitions (do not change)
#define OUTPUT__FORMAT_TEXT 0 // Outputs data as text
#define OUTPUT__FORMAT_BINARY 1 // Outputs data as binary float

// Select your startup output mode and format here!
int output_mode = 1; // OUTPUT__MODE_ANGLES;
int output_format = OUTPUT__FORMAT_TEXT;

// Select if serial continuous streaming output is enabled per default on startup.
#define OUTPUT__STARTUP_STREAM_ON false  // true or false

// If set true, an error message will be output if we fail to read sensor data.
// Message format: "!ERR: reading <sensor>", followed by "\r\n".
boolean output_errors = false;  // true or false

// Bluetooth
// You can set this to true, if you have a Rovering Networks Bluetooth Module attached.
// The connect/disconnect message prefix of the module has to be set to "#".
// (Refer to manual, it can be set like this: SO,#)
// When using this, streaming output will only be enabled as long as we're connected. That way
// receiver and sender are synchronzed easily just by connecting/disconnecting.
// It is not necessary to set this! It just makes life easier when writing code for
// the receiving side. The Processing test sketch also works without setting this.
// NOTE: When using this, OUTPUT__STARTUP_STREAM_ON has no effect!
#define OUTPUT__HAS_RN_BLUETOOTH false  // true or false


// SENSOR CALIBRATION
/*****************************************************************/
// How to calibrate? Read the tutorial at http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
// Put MIN/MAX and OFFSET readings for your board here!
// Accelerometer
// "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"

#define ACCEL_X_MIN ((float) -17528.00)
#define ACCEL_X_MAX ((float) 17272.00)
#define ACCEL_Y_MIN ((float) -16752.00)
#define ACCEL_Y_MAX ((float) 17384.00)
#define ACCEL_Z_MIN ((float) -17440.00)
#define ACCEL_Z_MAX ((float) 17592.00)

// Magnetometer (standard calibration mode)
// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"

#define MAGN_X_MIN ((float) -57.61)
#define MAGN_X_MAX ((float) 22.06)
#define MAGN_Y_MIN ((float) -3.00)
#define MAGN_Y_MAX ((float) 74.42)
#define MAGN_Z_MIN ((float) -42.46)
#define MAGN_Z_MAX ((float) 39.91)

// Magnetometer (extended calibration mode)
// Uncommend to use extended magnetometer calibration (compensates hard & soft iron errors)
// Use your own calibration values! The values below were for my board!
#define CALIBRATION__MAGN_USE_EXTENDED true
const float magn_ellipsoid_center[3] = { 220.450, -86.4332, -212.508 };
const float magn_ellipsoid_transform[3][3] = { { 0.980092, -0.00580039, -0.0175808 },{ -0.00580039, 0.964793, -0.00368069 },{ -0.0175808, -0.00368069, 0.984412 } };

// Gyroscope
// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z

#define GYRO_AVERAGE_OFFSET_X ((float) -34.74)
#define GYRO_AVERAGE_OFFSET_Y ((float) -24.41)
#define GYRO_AVERAGE_OFFSET_Z ((float) 4.27)

// DEBUG OPTIONS
/*****************************************************************/
// When set to true, gyro drift correction will not be applied
#define DEBUG__NO_DRIFT_CORRECTION false
// Print elapsed time after each I/O loop
#define DEBUG__PRINT_LOOP_TIME false


/*****************************************************************/
/****************** END OF USER SETUP AREA!  *********************/
/*****************************************************************/

#include <Wire.h> //this might not be needed but I was getting errors at one point for not having it

#define MPU9250_INT_PIN 4
#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration
// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))


// Gain for gyroscope
#define GYRO_GAIN 0.06957 // Same gain on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second

// DCM parameters
#define Kp_ROLLPITCH 0.02f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 0.01f
#define Ki_YAW .00001f

// Stuff
#define STATUS_LED_PIN 13  // Pin number of status LED
#define SD_CHIP_SELECT_PIN 38
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

// Moving average filter
const int numReadings = 60;
const int numReadings2 = 5;
int readIndex = 0; // the index of the current reading
int readIndex2 = 0;
/*******   Heading Filter Values ************/
float total_MAG_Heading = 0.0;
float average_MAG_Heading = 0.0;
float MAG_Heading_readings[numReadings];

/*******   Roll and Pitch Filter Values ************/
float total_roll = 0.0;
float average_roll = 0.0;
float roll_readings[numReadings2];

float total_pitch = 0.0;
float average_pitch = 0.0;
float pitch_readings[numReadings2];


// Sensor variables
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float accel_min[3];
float accel_max[3];

float magnetom[3];
float magnetom_min[3];
float magnetom_max[3];
float magnetom_tmp[3];

float gyro[3];
float gyro_average[3];
int gyro_num_samples = 0;

char values_param;
char format_param;
byte id[2];
char output_param;

// DCM variables
float MAG;
float MAG_Heading;
float heading_magnitude;
float Accel_Vector[3]= {0, 0, 0}; // Store the acceleration in a vector
float Gyro_Vector[3]= {0, 0, 0}; // Store the gyros turn rate in a vector
float Omega_Vector[3]= {0, 0, 0}; // Corrected Gyro_Vector data
float Omega_P[3]= {0, 0, 0}; // Omega Proportional correction
float Omega_I[3]= {0, 0, 0}; // Omega Integrator
float Omega[3]= {0, 0, 0};
float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};
float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Euler angles
float yaw;
float pitch;
float roll;
char command;
char read;
// DCM timing in the main loop
unsigned long timestamp;
unsigned long timestamp_old;
float G_Dt; // Integration time for DCM algorithm

// More output-state variables
boolean output_stream_on;
boolean output_single_on;
int curr_calibration_sensor = 0;
boolean reset_calibration_session_flag = true;
int num_accel_errors = 0;
int num_magn_errors = 0;
int num_gyro_errors = 0;

void read_sensors() {
	imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
	Read_Gyro(); // Read gyroscope
	Read_Accel(); // Read accelerometer
	Read_Magn(); // Read magnetometer
}

// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
// TODO re-init global vars?
void reset_sensor_fusion() {
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};

  read_sensors();
  timestamp = millis();
 
  // GET PITCH
  // Using y-z-plane-component/x-component of gravity vector
  pitch = -atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));
  // GET ROLL
  // Compensate pitch of gravity vector 
  Vector_Cross_Product(temp1, accel, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  // Normally using x-z-plane-component/y-component of compensated gravity vector
   //roll = atan2(accel[1], sqrt(accel[0] * accel[0] + accel[2] * accel[2]));
  // Since we compensated for pitch, x-z-plane-component equals z-component:
  roll = atan2(temp2[1], temp2[2]);
  // GET YAW
  Compass_Heading();
  //yaw = MAG_Heading;
  yaw = average_MAG_Heading;
  // Init rotation matrix
  init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
}

// Apply calibration to raw sensor readings
void compensate_sensor_errors() {
    // Compensate accelerometer error
    accel[0] = (accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
    accel[1] = (accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
    accel[2] = (accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;

    // Compensate magnetometer error
#if CALIBRATION__MAGN_USE_EXTENDED == true
    for (int i = 0; i < 3; i++)
      magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
    Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);
#else
    magnetom[0] = (magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
    magnetom[1] = (magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
    magnetom[2] = (magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;
#endif

    // Compensate gyroscope error
    gyro[0] -= GYRO_AVERAGE_OFFSET_X;
    gyro[1] -= GYRO_AVERAGE_OFFSET_Y;
    gyro[2] -= GYRO_AVERAGE_OFFSET_Z;
}

// Reset calibration session if reset_calibration_session_flag is set
void check_reset_calibration_session()
{
  // Raw sensor values have to be read already, but no error compensation applied

  // Reset this calibration session?
  if (!reset_calibration_session_flag) return;
  
  // Reset acc and mag calibration variables
  for (int i = 0; i < 3; i++) {
    accel_min[i] = accel_max[i] = accel[i];
    magnetom_min[i] = magnetom_max[i] = magnetom[i];
  }

  // Reset gyro calibration variables
  gyro_num_samples = 0;  // Reset gyro calibration averaging
  gyro_average[0] = gyro_average[1] = gyro_average[2] = 0.0f;
  
  reset_calibration_session_flag = false;
}

void turn_output_stream_on()
{
  output_stream_on = true;
  digitalWrite(STATUS_LED_PIN, HIGH);
}

void turn_output_stream_off()
{
  output_stream_on = false;
  digitalWrite(STATUS_LED_PIN, LOW);
}

// Blocks until another byte is available on serial port
char readChar()
{
  while (SerialPort.available() < 1) { } // Block
  return SerialPort.read();
}

void setup()
{
  pinMode (STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  pinMode(MPU9250_INT_PIN, INPUT_PULLUP);
  // Init serial output
  SerialPort.begin(OUTPUT__BAUD_RATE);
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

  // Use setGyroFSR() and setAccelFSR() to configure the
  // gyroscope and accelerometer full scale ranges.
  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  imu.setGyroFSR(2000); // Set gyro to 2000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(2); // Set accel to +/-2g
  // Note: the MPU-9250's magnetometer FSR is set at 
  // +/- 4912 uT (micro-tesla's)

  // setLPF() can be used to set the digital low-pass filter
  // of the accelerometer and gyroscope.
  // Can be any of the following: 188, 98, 42, 20, 10, 5
  // (values are in Hz).
  imu.setLPF(188); // Set LPF corner frequency to 5Hz
  // The sample rate of the accel/gyro can be set using
  // setSampleRate. Acceptable values range from 4Hz to 1kHz
  imu.setSampleRate(1000); // Set sample rate to 10Hz

  // Likewise, the compass (magnetometer) sample rate can be
  // set using the setCompassSampleRate() function.
  // This value can range between: 1-100Hz
  imu.setCompassSampleRate(100); // Set mag rate to 10Hz
  imu.dmpBegin(DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_CAL_GYRO, 200);
  // Init sensors
  delay(50);  // Give sensors enough time to start
  //I2C_Init();
  //Accel_Init();
  //Magn_Init();
  //Gyro_Init();
  imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
  imu.dmpUpdateFifo();
  // Read sensors, init DCM algorithm
  delay(20);  // Give sensors enough time to collect data
  reset_sensor_fusion();
  //SerialPort.println();
  //SerialPort.println("******** IMU Setup Complete **************");

}

// Main loop
void loop()
{
	// Read incoming control messages
	if (SerialPort.available() >= 2)
	{
		if (SerialPort.read() == '#') // Start of new control message
		{
			int command = SerialPort.read(); // Commands
			if (command == 'f') // request one output _f_rame
				output_single_on = true;
			else if (command == 's') // _s_ynch request
			{
				// Read ID
				byte id[2];
				id[0] = readChar();
				id[1] = readChar();

				// Reply with synch message
				SerialPort.print("#SYNCH");
				SerialPort.write(id, 2);
				SerialPort.println();
			}
			else if (command == 'o') // Set _o_utput mode
			{
				char output_param = readChar();
				if (output_param == 'n')  // Calibrate _n_ext sensor
				{
					curr_calibration_sensor = (curr_calibration_sensor + 1) % 3;
					reset_calibration_session_flag = true;
				}
				else if (output_param == 't') // Output angles as _t_ext
				{
					output_mode = OUTPUT__MODE_ANGLES;
					output_format = OUTPUT__FORMAT_TEXT;
				}
				else if (output_param == 'b') // Output angles in _b_inary format
				{
					output_mode = OUTPUT__MODE_ANGLES;
					output_format = OUTPUT__FORMAT_BINARY;
				}
				else if (output_param == 'c') // Go to _c_alibration mode
				{
					output_mode = OUTPUT__MODE_CALIBRATE_SENSORS;
					reset_calibration_session_flag = true;
				}
				else if (output_param == 's') // Output _s_ensor values
				{
					char values_param = readChar();
					char format_param = readChar();
					if (values_param == 'r')  // Output _r_aw sensor values
						output_mode = OUTPUT__MODE_SENSORS_RAW;
					else if (values_param == 'c')  // Output _c_alibrated sensor values
						output_mode = OUTPUT__MODE_SENSORS_CALIB;
					else if (values_param == 'b')  // Output _b_oth sensor values (raw and calibrated)
						output_mode = OUTPUT__MODE_SENSORS_BOTH;

					if (format_param == 't') // Output values as _t_text
						output_format = OUTPUT__FORMAT_TEXT;
					else if (format_param == 'b') // Output values in _b_inary format
						output_format = OUTPUT__FORMAT_BINARY;
				}
				else if (output_param == '0') // Disable continuous streaming output
				{
					turn_output_stream_off();
					reset_calibration_session_flag = true;
				}
				else if (output_param == '1') // Enable continuous streaming output
				{
					reset_calibration_session_flag = true;
					turn_output_stream_on();
				}
				else if (output_param == 'e') // _e_rror output settings
				{
					char error_param = readChar();
					if (error_param == '0') output_errors = false;
					else if (error_param == '1') output_errors = true;
					else if (error_param == 'c') // get error count
					{
						SerialPort.print("#AMG-ERR:");
						SerialPort.print(num_accel_errors); SerialPort.print(",");
						SerialPort.print(num_magn_errors); SerialPort.print(",");
						SerialPort.println(num_gyro_errors);
					}
				}
			}
#if OUTPUT__HAS_RN_BLUETOOTH == true
			// Read messages from bluetooth module
			// For this to work, the connect/disconnect message prefix of the module has to be set to "#".
			else if (command == 'C') // Bluetooth "#CONNECT" message (does the same as "#o1")
				turn_output_stream_on();
			else if (command == 'D') // Bluetooth "#DISCONNECT" message (does the same as "#o0")
				turn_output_stream_off();
#endif // OUTPUT__HAS_RN_BLUETOOTH == true
		}
		else
		{
		} // Skip character
	}

	// Time to read the sensors again?
	if ((millis() - timestamp) >= OUTPUT__DATA_INTERVAL)
	{
		timestamp_old = timestamp;
		timestamp = millis();
		if (timestamp > timestamp_old)
			G_Dt = (float)(timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
		else G_Dt = 0;

		// Update sensor readings
		read_sensors();

		if (output_mode == OUTPUT__MODE_CALIBRATE_SENSORS)  // We're in calibration mode
		{
			check_reset_calibration_session();  // Check if this session needs a reset
			if (output_stream_on || output_single_on) output_calibration(curr_calibration_sensor);
		}
		else if (output_mode == OUTPUT__MODE_ANGLES)  // Output angles
		{
			// Apply sensor calibration
			compensate_sensor_errors();

			// Run DCM algorithm
			Compass_Heading(); // Calculate magnetic heading
			Matrix_update();
			Normalize();
			Drift_correction();
			Euler_angles();

			if (output_stream_on || output_single_on) output_angles();
		}
		else  // Output sensor values
		{
			if (output_stream_on || output_single_on) output_sensors();
		}

		output_single_on = false;

#if DEBUG__PRINT_LOOP_TIME == true
		SerialPort.print("loop time (ms) = ");
		SerialPort.println(millis() - timestamp);
#endif
	}
#if DEBUG__PRINT_LOOP_TIME == true
	else
	{
		SerialPort.println("waiting...");
	}
#endif
}