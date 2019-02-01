// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc4579.Curiosity.subsystems;

import org.usfirst.frc4579.Curiosity.RobotMap;
import org.usfirst.frc4579.Curiosity.commands.*;
import org.usfirst.frc4579.filters.FirstOrderLPF;
import org.usfirst.frc4579.Curiosity.Robot;

import edu.wpi.first.wpilibj.command.Subsystem;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class DriveTrain extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final SpeedController frontLeft = RobotMap.driveTrainfrontLeft;
    private final SpeedController backLeft = RobotMap.driveTrainbackLeft;
    private final SpeedControllerGroup leftMotors = RobotMap.driveTrainleftMotors;
    private final SpeedController backRight = RobotMap.driveTrainbackRight;
    private final SpeedController frontRight = RobotMap.driveTrainfrontRight;
    private final SpeedControllerGroup rightMotors = RobotMap.driveTrainrightMotors;
    private final DifferentialDrive robotDrive = RobotMap.driveTrainrobotDrive;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public final double TURN_SPEED = 0.12;

    final double BASELINE = 19.75; // inches.
    final double MAXSPEED = 93.0;  // inches per second.
    
    private FirstOrderLPF vLeftLPF = new FirstOrderLPF(0.7);
    private FirstOrderLPF vRiteLPF = new FirstOrderLPF(0.7);
    private FirstOrderLPF gyro     = new FirstOrderLPF(0.9);
    
    public double angle = 0.0;

    public boolean isDriveInverted = false;
    
 // Return true if the motors are commanded to zero.
    public boolean isNotMoving() {   
    	return (leftMotors.get() == 0.0) && (rightMotors.get() == 0.0);
    }
    
    public void joeyStickDrive(double x, double y) { //The finest drive code known to man.
		
    	//Read the gyro and the driveStick.
		double gz = Robot.measurement.getAngle();
		double frwd = x;		//forward-back driveStick, speed control.
		double turn = -y;	    //left-right driveStick, turn control, left turn is positive.

		//Lower limits for the driveStick, stop the motors.
		if (Math.abs(turn) < 0.05 && Math.abs(frwd) < 0.05) { 
			turn = 0.0;
			frwd = 0.0;
			gz = 0.0;
		}
		
		//Decrease the low speed sensitivities of the driveStick.
		double frwd2 = Math.signum(frwd) * Math.pow(Math.abs(frwd), 1.5);
		double turn2 = Math.signum(turn) * Math.pow(Math.abs(turn), 2.0);
//		JGH Try this alternate version that takes the limits into account.
//		double frwd2 = Math.signum(frwd) * Math.pow(Math.abs(frwd - limit), 1.5);
//		double turn2 = Math.signum(turn) * Math.pow(Math.abs(turn - limit), 2.0);
		
		
		//Limit the control amount at high and low speeds, to avoid spinouts.
		double maxSens = 0.55;
		double minSens = 0.2;
		double sensitivity = maxSens - Math.abs(frwd2) * (maxSens - minSens);
		turn2 = turn2 * sensitivity;
		
		// Sets the speed scaling of the robot
		double speed = (-Robot.oi.joystick.getThrottle() + 1.0) / 2.0;
		
		//Low pass filter the speed settings to the drive motors.
		double vLeft = vLeftLPF.filter(frwd2 + turn2 / 2.0) * speed;
		double vRite = vRiteLPF.filter(frwd2 - turn2 / 2.0) * speed;
		
		//Calculate the expected rotation rate.  93 in/sec (extrapolated full speed) converts the driveStick 
		//numbers to an expected speed value. The final equation is omega = (SpeedRite - SpeedLeft)/baseline.  
		//omega is rotation in deg/sec.
		double omega = Math.toDegrees((vRite - vLeft) * 93.0 / BASELINE); 
		
		//Calculate the two wheel correction factor.
		double correction  = (omega - gz) * 0.008 / 2.0;
		double vRite2 = vRite + correction;
		double vLeft2 = vLeft - correction;

		
		//Normalize the wheel speeds to stay within +/-1.0;
		double magMax = Math.max(Math.abs(vRite2), Math.abs(vLeft2));
		if (magMax > 1.0) {
			vRite2 /= magMax;
			vLeft2 /= magMax;
		}

		//Set the two motor speeds.
		rightMotors.set(vRite2);
		leftMotors.set(vLeft2);
		
		//Prints out info from driving control loop
//		if (Math.abs(turn) >= 0.05 || Math.abs(frwd) >= 0.05) { 
//		System.out.printf("%6.3f  %6.3f  %6.3f  %6.3f  %6.3f  %6.3f  %7.6f  %7.6f  %6.3f\n", 
//				   frwd, turn, sensitivity, speed, vLeft, vRite, omega, gz, correction);
//		}
		
		return;
	}  // End of joeyStickDrive().
    
    // This is an alternative version of driving that adds the use of the robot angle
    // as measured in the measure1 subsystem to be used for steering.  This version
    // uses both the angle rate and the angle as measured from the joystick to control
    // the robot steering.  Therefore, it replaces the old driveStraight() method.  
    // This method can be used with both joystick and autonomous inputs.
    // @param:  frwd is positive ahead, turn is positive to the left.
    public void SmoothDrive(double frwd, double turn) { //The finest-er drive code known to man.
		final double limit = 0.05;
 
    	//Get the gyro data.
		double gz = gyro.filter(Robot.measurement.angleRate);
		double ga = Robot.measurement.angle;
		double gzMax = Robot.measurement.angleRateMax;
		
		//Lower limits for the inputs, stop the motors. (This is for joysticks.)
		if (Math.abs(turn) < limit && Math.abs(frwd) < limit) { 
			turn = 0.0;
			frwd = 0.0;
		}
		
		//Decrease the low speed sensitivities of the inputs.
//		double frwd2 = Math.signum(frwd) * Math.pow(Math.abs(frwd), 1.5);
//		double turn2 = Math.signum(turn) * Math.pow(Math.abs(turn), 2.0);
//		JGH Try this alternate version that takes the limits into account.
		double frwd2 = Math.signum(frwd) * Math.pow(Math.abs(frwd - limit), 1.5);
		double turn2 = Math.signum(turn) * Math.pow(Math.abs(turn - limit), 2.0);
		
		//Limit the turn control amount at high and low speeds, to avoid spinouts.
		double maxSens = 0.55;
		double minSens = 0.2;
		double sensitivity = maxSens - Math.abs(frwd2) * (maxSens - minSens);
		turn2 = turn2 * sensitivity;  //Limits turn2 to +/- 0 -> maxSens.
		
		// Sets the speed scaling of the robot
		double speed = (-Robot.oi.joystick.getThrottle() + 1.0) / 2.0;
		
		//Low pass filter the speed settings to the drive motors.
		double vLeft = vLeftLPF.filter(frwd2 + turn2 / 2.0) * speed;
		double vRite = vRiteLPF.filter(frwd2 - turn2 / 2.0) * speed;
		
		//Calculate the expected rotation rate and changing angle.  MAXSPEED 
		//converts the input numbers to an expected speed value. 
		//The final equation is omega = (SpeedRite - SpeedLeft)/baseline.  
		//omega is rotation in deg/sec.
		double omega = Math.toDegrees((vRite - vLeft) / 2.0 * MAXSPEED / BASELINE); 
		angle += omega * Robot.measurement.mpuDeltaT;
		
		//Calculate the two wheel correction factor, expected - actual omega.
		//double correction  = (omega - gz) * 0.008 / 2.0;
	    double correction = ((omega - gz) * 0.006 + (angle - ga) * 0.28) / 2.0;
		double vRite2 = vRite + correction;
		double vLeft2 = vLeft - correction;
		
		//Normalize the wheel speeds to stay within +/-1.0;
		double magMax = Math.max(Math.abs(vRite2), Math.abs(vLeft2));
		if (magMax > 1.0) {
			vRite2 /= magMax;
			vLeft2 /= magMax;
		}
		
		//Set the two motor speeds.
		rightMotors.set(vRite2);
		leftMotors.set(vLeft2);
		System.out.printf("%6.3f  %6.3f  %6.3f  %6.3f  %6.3f  %7.3f  %7.3f  %8.3f  %8.3f  %6.3f\n", 
						   frwd, turn, sensitivity, vLeft, vRite, omega, gz, angle, ga, correction);
		return;
	}  // End of smoothDrive().
        

    public void driveStraight(double speed) {
	    double halfCorrection = ((Robot.measurement.getAngleRate() * .006) + (Robot.measurement.getAngle() * .028)) /2.0;
	    leftMotors.set(speed + halfCorrection);
	    rightMotors.set(speed - halfCorrection);
    }
    
    public void driveStraightReference(double speed, double reference){
	    double halfCorrection = ((Robot.measurement.getAngleRate() * .006) + ((Robot.measurement.getAngle()-reference) * .028)) /2.0;
	    leftMotors.set(speed + halfCorrection);
	    rightMotors.set(speed - halfCorrection); 
    }
    
    // Drive method that uses joeyStickDrive and allows you to invert your direction and drive backwards
    public void competitionDrive(){
    	if(!isDriveInverted){
    		Robot.driveTrain.joeyStickDrive(-Robot.oi.joystick.getY(), Robot.oi.joystick.getX());
    	}
    	else{
    		Robot.driveTrain.joeyStickDrive(Robot.oi.joystick.getY(), -Robot.oi.joystick.getX());
    	}
    }
    
    public void invertDrive(){
    	isDriveInverted = !isDriveInverted;
    }
    
    public void stop(){
    	leftMotors.stopMotor();
    	rightMotors.stopMotor();
    }
    
    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        setDefaultCommand(new drive());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}

