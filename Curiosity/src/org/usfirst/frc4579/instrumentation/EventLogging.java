/**********************************************************************
 * EVENT LOG
 * This class allows the coder to leave a trail of time-tagged
 * "bread crumbs" (events) for debug purposes.  These are written
 * to a file on the roboRIO (see DebugTextFile class).
 **********************************************************************/

package org.usfirst.frc4579.instrumentation;

public class EventLogging extends Instrumentation {
	
	/*********************************************************************************
	 * NORMAL EVENTS document "normal" execution events, e.g, the beginning and end of
	 * major functional flows.
	 *********************************************************************************/
	public enum NORMALEVENTS {
						START_INITIALIZE_COMMAND,   // Outer level command events (NORMAL)
						END_INITIALIZE_COMMAND,
						START_EXECUTE_COMMAND,
						END_EXECUTE_COMMAND,
						START_COMMAND_IS_FINNISHED,
						END_COMMAND_IS_FINNISHED,
						START_COMMAND_END,
						END_COMMAND_END,
						START_COMMAND_INTERRUPTED,
						END_COMMAND_INTERRUPTED,
						CAMERA_INITIALIZED,          // Vision processing (NORMAL)
						CAMERA_START_IMAGE_GRAB,
						CAMERA_END_IMAGE_GRAB,
						START_TRACK_TARGET, 
						END_TRACK_TARGET,
						START_TRACK_TARGET_GET_IMAGE,
						END_TRACK_TARGET_GET_IMAGE,
						START_TRACK_TARGET_GRIP_PROCESSING_PIPELINE,
						END_TRACK_TARGET_GRIP_PROCESSING_PIPELINE,
						START_TRACK_TARGET_PROCESSING_CONTOURS,
						END_TRACK_TARGET_PROCESSING_CONTOURS,
						START_TRACK_TARGET_DRAWING_CONTOURS,
						END_TRACK_TARGET_DRAWING_CONTOURS,
						START_TRACK_TARGET_WRITING_IMAGE,
						END_TRACK_TARGET_WRITING_IMAGE,
						START_MPU_INIT,
						END_MPU_INIT,
						START_UPDATE_MPU_DATA,
						END_UPDATE_MPU_DATA

	};
	
	/*********************************************************************************
	 * INTERESTING EVENTS document, as an example, the results of lower-level flows 
	 * of control.
	 *********************************************************************************/
	public enum INTERESTINGEVENTS {
						
						MEASUREMENT_DATA
	};

	/*********************************************************************************
 	 * BAD EVENTS should not happen and should be debugged.
 	 *********************************************************************************/
	public enum BADEVENTS {
						TARGET_TRACKER_UNHANDLED_EXCEPTION
	};

	private static boolean        logAvailable = false;
	private static DebugTextFile  eventLogFile = new DebugTextFile("Events", false, "", 30000);

	private static EventLogging   eventsObj;
	
	// Constructor
	public EventLogging () {

		// Create the directory to hold the events file.
		super();
		logAvailable = instrumentationAvailable();
		logAvailable = false;  //Added to avoid logging failures. JGH
	}

	// Invoke the constructor to set up the instrumentation file structure.
	static {
		eventsObj = new EventLogging();
	}	

	/***************************************************************************
	 * This method returns true if event logging is available.  It would not
	 * be available if the instrumentation data directory structure could not
	 * be created.
	 ***************************************************************************/
	public static boolean loggingAvailable() {
		return logAvailable;
	}

	/***************************************************************************
	 * This method logs the specified event onto the eventLog.
	 ***************************************************************************/
	private static synchronized void logEvent (String event) {

		String eventStr = String.format("%10.6f", timeNow()) + '\t' + event + '\t';
		
		eventLogFile.write(eventStr);

	}

	/***************************************************************************
	 * This method logs a NORMAL event.
	 ***************************************************************************/
	public static synchronized void logNormalEvent (NORMALEVENTS event, String auxData) {

		if (logAvailable) 
			logEvent ("NORMAL      \t" + event.name() + "\t" + auxData);

	}

	/***************************************************************************
	 * This method logs an INTERESTING event.
	 ***************************************************************************/
	public static synchronized void logInterestingEvent (INTERESTINGEVENTS event, String auxData) {

		if (logAvailable) 
			logEvent ("INTERESTING\t" + event.name() + "\t" + auxData);

	}

	/***************************************************************************
	 * This method logs a BAD event.
	 ***************************************************************************/
	public static synchronized void logBadEvent (BADEVENTS event, String auxData) {

		if (logAvailable) 
			logEvent ("BAD         \t" + event.name() + "\t" + auxData);
	}
}

