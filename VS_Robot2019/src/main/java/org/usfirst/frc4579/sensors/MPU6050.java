package org.usfirst.frc4579.sensors;


import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

// This class is the interface to the MPU 6050 sensor.
// It initializes the sensor at the time the constructor is instantiated, and 
// then reads the sensor when requested.  The sensor provides many values which
// are available as class constants, or by use of getters.
// There is an internal setting for the I2C bus address of the device, 
// and for setting the number of calibration steps to use, and
// for when the sensor is mounted upside down, which reverses the readings.  
// There are also values for determining if there is a good reading, 
// and if the sensor is accessible.
//
// There is a constant that needs to be set to adjust the readings in case the sensor
// is mounted upside down.  The logic assumes that the sensor +X axis is pointing 
// forward on the robot.  For other orientations, adjust the code in readAxes().

public class MPU6050 extends SensorBase {
	// Define the public and private data values from the sensor.  
	public double accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
	public double gyroZMax = 0.0;
	public double deltaT = 0.0;
	public boolean goodSensor = false;
	
	private static final byte   deviceAddress         = 0x68;
	private static final int    numCalibrationSamples = 100;
	private static final double	upsideDown			  = -1.0;  //-1.0 for upside down mounting.

	// Instantiate channels.
	private I2C MPU;
	// Initialize delta time variables.
	private double time = Timer.getFPGATimestamp();
	private double lastTime = time;
	// Acceleration configuration data.
	public  enum ACCELFULLSCALE { ACCEL2G  , ACCEL4G  , ACCEL8G   , ACCEL16G };
	public  enum GYROFULLSCALE  { DEGSEC250, DEGSEC500, DEGSEC1000, DEGSEC2000};
	// Acceleration axes configuration data.
	private final int[]    accelFullScaleRegSettings = {0x00, 0x08, 0x10, 0x18}; // Bits 3 & 4 of REGISTER_ACCEL_CONFIG
	private final double[] accelScaleFactors         = {16384.0, 8192.0, 4096.0, 2048.0};
	private final int      accelRegConfigValue;
	private final double   accelScaleFactor;
	// Gyro axes configuration data.
	private final int[]    gyroFullScaleRegSettings = {0x00, 0x08, 0x10, 0x18}; // Bits 3 & 4 of REGISTER_GYRO_CONFIG
	private final double[] gyroScaleFactors         = {131.0, 65.5, 32.8, 16.4};
	private final int      gyroRegConfigValue;
	private final double   gyroScaleFactor;
	//Define registers to be used
	@SuppressWarnings("unused")
	private static final int 
		REGISTER_SELF_TEST_X       = 0x0D,
		REGISTER_SELF_TEST_Y       = 0x0E,
		REGISTER_SELF_TEST_Z       = 0x0F,
		REGISTER_SELF_TEST_A       = 0x10,
		REGISTER_SAMPLE_RATE       = 0x19,
		REGISTER_CONFIG            = 0x1A,
		REGISTER_GYRO_CONFIG       = 0x1B,
		REGISTER_ACCEL_CONFIG      = 0x1C,
		REGISTER_FIFO_ENABLE       = 0x23,
		REGISTER_INTERRUPT_ENABLE  = 0x38,
		REGISTER_INTERRUPT_STATUS  = 0x3A,
		REGISTER_ACCEL             = 0x3B,
		REGISTER_TEMP              = 0x41,
		REGISTER_GYRO              = 0x43,
		REGISTER_SIGNAL_PATH_RESET = 0x68, // bit 0 =  Temp, 1 = accel, 2 = gyro
		REGISTER_PWRMGMT_1         = 0x6B, 
		REGISTER_PWRMGMT_2         = 0x6C,
	 	REGISTER_WHO_AM_I		   = 0x75;
	
	// Calibration factors for the sensor axes.
	private double accelXoffset = 0.0;
	private double accelYoffset = 0.0;
	private double accelZoffset = 0.0;
	private double gyroXoffset  = 0.0;
	private double gyroYoffset  = 0.0;
	private double gyroZoffset  = 0.0;
	
	// Constructor.  The two inputs are the desired full scale readings for the accelerometer
	// and the gyro axes.  These are defined types and enumerated constants above.
	// Constructor..................
	public MPU6050 (ACCELFULLSCALE accelFullScale, GYROFULLSCALE gyroFullScale)  {
		accelRegConfigValue = accelFullScaleRegSettings[accelFullScale.ordinal()];
		accelScaleFactor    = accelScaleFactors        [accelFullScale.ordinal()]; // / GsToAccel;
		gyroRegConfigValue  = gyroFullScaleRegSettings [gyroFullScale .ordinal()];
		gyroScaleFactor     = gyroScaleFactors         [gyroFullScale .ordinal()];
		// Set up the I2C port.
		MPU = new I2C(I2C.Port.kOnboard, (int)deviceAddress);
		//private I2C MPU = new I2C(I2C.Port.kOnboard, (int)deviceAddress);

		if (MPU.addressOnly()) {  //Test bus address, true if aborted transaction.
			goodSensor = false;
		}
		goodSensor = init();
	}  // End of Constructor().
 
	
	// Primary functional method for the MPU sensor.  The calling program should
	// execute this method, and if it returns true, access the data elements
	// directly, like: count = myMPU.accelX; ...or use the getters below.
	// If it returns false, then the sensor data is not valid.
	// Initializes the MPU with predefined settings.
	public boolean readAxes() {
//		System.out.println("Reading axes");
		
		//Get the raw data.
		if (readRawData()) {
			//Convert to scaled values and subtract the offsets.
			//Assume the +X axis is forward.
			accelX = (accelX / accelScaleFactor - accelXoffset);
			accelY = (accelY / accelScaleFactor - accelYoffset) * upsideDown;
			accelZ = (accelZ / accelScaleFactor - accelZoffset) * upsideDown;
			gyroX  = (gyroX  / gyroScaleFactor  - gyroXoffset)  * upsideDown;
			gyroY  = (gyroY  / gyroScaleFactor  - gyroYoffset)  * upsideDown;
			gyroZ  = (gyroZ  / gyroScaleFactor  - gyroZoffset)  * upsideDown;
			if (Math.abs(gyroZ) > gyroZMax) gyroZMax = Math.abs(gyroZ);
			return true;
		}
		System.out.println("In readAxes() we got a bad read from RawData().");
		accelX = 0.0;
		accelY = 0.0;
		accelZ = 0.0;
		gyroX  = 0.0;
		gyroY  = 0.0;
		gyroZ  = 0.0;
		goodSensor = false;
		return false;
	}  // End of ReadAxes().
	
	public double getAccelX() {
		return accelX;
	}
	
	public double getAccelY() {
		return accelY;
	}
	
	public double getAccelZ() {
		return accelZ;
	}
	
	public double getGyroX() {
		return gyroX;
	}
	
	public double getGyroY() {
		return gyroY;
	}
	
	public double getGyroZ() {
		return gyroZ;
	}
	
	public double getGyroZMax() {
		return gyroZMax;
	}
	
	public double getDeltaT() {
		return deltaT;
	}
	
	public boolean getGoodSensor() {
		return goodSensor;
	}

	
	// Read the raw sensor data into the public variables.
	private boolean readRawData() {
//		System.out.println("Getting raw data");
		byte[] buffer = new byte[14];
		int dataReady = 0;
		boolean timeout = true;
		Timer t = new Timer();
			
		t.reset();
		t.start();			
		do {  //Wait for data ready status.
			MPU.read(REGISTER_INTERRUPT_STATUS, 1, buffer);
			dataReady = buffer[0] & 0x01;
			timeout = (t.get() > 0.01);
		} while ((dataReady == 0) & !timeout) ;
		
		t.stop();
		
		if (!timeout) {  //Get all the data bytes.
			MPU.read(REGISTER_ACCEL, 14, buffer);
			// Compute deltaT.
			time = Timer.getFPGATimestamp();
			deltaT = lastTime - time;
			lastTime = time;
			accelX = ((int)buffer[0]  << 8) | (buffer[1]  & 0xff);
			accelY = ((int)buffer[2]  << 8) | (buffer[3]  & 0xff);
			accelZ = ((int)buffer[4]  << 8) | (buffer[5]  & 0xff);
			gyroX  = ((int)buffer[8]  << 8) | (buffer[9]  & 0xff);
			gyroY  = ((int)buffer[10] << 8) | (buffer[11] & 0xff);
			gyroZ  = ((int)buffer[12] << 8) | (buffer[13] & 0xff);
			return true;
		}
		else { //We timed out.
			System.out.println("In readRawData(), sensor read timed out.");
			return false;
		}
	}  //End of readRawData().
		
	// Calibrate all the axes.  Usually done as part of initialization.
	private boolean calibrate() {
		double sumAccelX = 0.0;
		double sumAccelY = 0.0;
		double sumAccelZ = 0.0;
		double sumGyroX  = 0.0;
		double sumGyroY  = 0.0;
		double sumGyroZ  = 0.0;
		
		// Compute calibration data, Method 1.
		// For the accelerometers, we just take
		// the average of a large number of readings.  This can be used as a correction
		// factor in the returned values.
		// For the gyros, we have to compute offset and drift.  We take an average of a
		// small number of readings, wait one second, and take another average.  These
		// values are used to calculate the drift.  In this Method, no drift is calculated.
		for(int i = 0; i < numCalibrationSamples; i++) {
			if (readAxes()) {
				sumAccelX += accelX;
				sumAccelY += accelY;
				sumAccelZ += accelZ;
				sumGyroX  += gyroX;
				sumGyroY  += gyroY;
				sumGyroZ  += gyroZ;
			}  // End of summation.
			else {  //Not reading the chip correctly.
				System.out.println("In calibration(), not reading sensor correctly.");
				accelXoffset = 0.0;
				accelYoffset = 0.0;
				accelZoffset = 0.0;
				gyroXoffset  = 0.0;
				gyroYoffset  = 0.0;
				gyroZoffset  = 0.0;
				return false;
			}
			accelXoffset = sumAccelX / numCalibrationSamples;
			accelYoffset = sumAccelY / numCalibrationSamples;
			accelZoffset = sumAccelZ / numCalibrationSamples;
			gyroXoffset  = sumGyroX  / numCalibrationSamples;
			gyroYoffset  = sumGyroY  / numCalibrationSamples;
			gyroZoffset  = sumGyroZ  / numCalibrationSamples;
		}  	
		return true;
	}  // End of calibrate().
	
	
	// Initialize the MPU6050 Sensor Chip.
	private boolean init() {
		byte[] buffer = new byte[1];
		// Verify connection to the chip.
		MPU.read(REGISTER_WHO_AM_I, 1, buffer);
		if (buffer[0] != (byte)0x68) {
			System.out.printf("In init() MPU6050 Who_Am_I reads: %x\n", buffer[0]);
			accelX = 0.0;
			accelY = 0.0;
			accelZ = 0.0;
			gyroX = 0.0;
			gyroY = 0.0;
			gyroZ = 0.0;
			return false;
		}  
		// Good communication with the chip.
		// Set up the chip.
		MPU.write(REGISTER_SAMPLE_RATE      , 7);  // Sample rate divider.
		MPU.write(REGISTER_CONFIG           , 6);  // No external sync, DLPF mode 6.
		MPU.write(REGISTER_GYRO_CONFIG , gyroRegConfigValue);
		MPU.write(REGISTER_ACCEL_CONFIG, accelRegConfigValue);
		MPU.write(REGISTER_PWRMGMT_1        , 1);  // PLL with X axis gyroscope reference
		MPU.write(REGISTER_FIFO_ENABLE      , 0);  // Disable FIFO.
		MPU.write(REGISTER_INTERRUPT_ENABLE , 1);  // Interrupt enable
    	return true;
	}  // End of init().


	@Override
	public void initSendable(SendableBuilder builder) {
		// TODO Auto-generated method stub
	}

}  // End of MPU6050 class definition.
