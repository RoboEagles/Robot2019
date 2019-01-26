package org.usfirst.frc4579.instrumentation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*******************************************************************************
 * FRC PC DASHBOARD
 * 
 * This class encapsulates the functionality of the FRC PC (LabView) Dashboard.
 * 
 *******************************************************************************/

public class FRCPCDashboard {

	/* 
	 * Writing to Basic tab of the FRC PC (Labview) Dashboard: 
	 * 
	 *    The strings are labeled top-to-bottom, left-to-right from "DB/String 0" to "DB/String 9". Each String 
	 *    field can display at least 21 characters (exact number depends on what characters).
	 * 
	 *    SmartDashboard.putString("DB/String 0", "My 21 Char TestString");
	 * 
	 * Reading string data entered on the Basic tab of the FRC PC (Labview) Dashboard:
	 * 
	 *    String dashData = SmartDashboard.getString("DB/String 0", "myDefaultData");
	 */
	
	// Insert Basic tab DB methods here.
	
	/*
	 * Buttons and LEDs on the Basic tab of the FRC PC (Labview) Dashboard:
	 * 
	 *    The Buttons and LEDs are boolean values and are labeled top-to-bottom from "DB/Button 0" to "DB/Button 3" 
	 *    and "DB/LED 0" to "DB/LED 3". The Buttons are bi-directional, the LEDs are only able to be written from 
	 *    the Robot and read on the Dashboard. 
	 *    
	 *    To write to the Buttons or LEDs: SmartDashboard.putBoolean("DB/Button 0", true);
	 *    
	 *    To read the Buttons: boolean buttonValue = SmartDashboard.getBoolean("DB/Button 0", false); // default value of false.
	 */
	
	// Insert Basic tab button and LED methods here.
	
	/*
	 * Sliders on the Basic tab of the FRC PC (Labview) Dashboard:
	 * 
	 *    The Sliders are bi-directional analog (double) controls/indicators with a range from 0 to 5. 
	 *    
	 *    To write to these indicators: SmartDashboard.putNumber("DB/Slider 0", 2.58);
	 *    
	 *    To read values from the Dashboard into the robot program: SmartDashboard.getNumber("DB/Slider 0", 0.0); // default value of 0
	 *    
	 */
	
	// Insert Basic tab slider methods here.

}
