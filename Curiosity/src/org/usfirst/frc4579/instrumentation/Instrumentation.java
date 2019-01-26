/*****************************************************************************
 * INSTRUMENTATION
 * 
 * This class creates a "runs" data directory into which debug data can be
 * saved for a particular run.  A new folder is created in the "runs"
 * directory to hold the data for a particular run, with the name of
 * the folder containing the date and time that the run began.  The inheriting 
 * subclass responsible for writing a particular kind of debug data has access 
 * to this folder path via the dataDirectoryName method.
 *****************************************************************************/

package org.usfirst.frc4579.instrumentation;

import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.time.temporal.ChronoUnit;
import java.util.Date;
import java.util.Locale;

import edu.wpi.first.wpilibj.Timer;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.attribute.BasicFileAttributes;

public abstract class Instrumentation {

	private static String  runDataDir = new String();
	private static boolean instrAvailable;
	private static String  runsPath;
	
	// Constructor
	Instrumentation () {

		synchronized(this) {

			// Has this run's instrumentation directory already been created?
			instrAvailable = !runDataDir.isEmpty();

			// if not
			if (!instrAvailable) {

				// Try to create a "runs" directory file object.
				String mainPath = System.getProperty("user.home");

				//System.out.println(mainPath);

				instrAvailable  = false;

				runsPath = mainPath + "/runs";

				File fileDir1 = new File(runsPath);

				// if the directory doesn't already exist try to create it
				if (!fileDir1.exists()) 
					instrAvailable = fileDir1.mkdirs();

				// if the "runs" directory exists
				if (instrAvailable) {

					// Try to create the unique directory for this run
					SimpleDateFormat dateFormat = new SimpleDateFormat ("yyyy.MM.dd__hh.mm");
					Date date                   = new Date();
					runDataDir                  = fileDir1.getAbsolutePath() + "/" + dateFormat.format(date);
					File fileDir2               = new File(runDataDir);

					instrAvailable = fileDir2.mkdir();
				}
			}
		}
	}
	
	// Recursive method to delete the contents of a file directory and then the
	// directory itself.
	private static void deleteDir(File file) {
	    File[] contents = file.listFiles();
	    if (contents != null) {
	        for (File f : contents) {
	            deleteDir(f);
	        }
	    }
	    file.delete();
	}
	
	// Automatically clean out old data directories that are at least daysOld in age.
	public static void deleteOldDataDirectories (int daysOld) {

		if (!instrAvailable) return;
		
		// Create a file object for the "runs" directory.
		File runs = new File(runsPath);

		if (runs.exists())
		{
			// Format of file date attribute
			SimpleDateFormat dateFormat = new SimpleDateFormat ("yyyy-MM-dd HH:mm:ss");
			
			// Create a Date object for the current date and time.
			Date now = new Date();
			
			// Get a list of run data directories.
			String[] runDirs = runs.list();

			// For each one
			for (int i=0; i < runDirs.length; i++) {
				
				// Create a File object for directory i
				File runDir = new File(runsPath + "/" + runDirs[i]);

				// Set up for getting file attributes.
				Path path = Paths.get(runsPath + "/" + runDirs[i]);
				BasicFileAttributes attr;

				try {
					// Read the file attributes.
					attr = Files.readAttributes(path, BasicFileAttributes.class);
					String cDate = attr.creationTime().toString().replace('T', ' ').substring(0, 19);
					
					// Create a Date object from the creation data.
					Date creationDate = dateFormat.parse(cDate);
					
					// if the file directory is considered old
					long numDaysOld = ChronoUnit.DAYS.between(creationDate.toInstant(), now.toInstant());
					
					if (numDaysOld > daysOld) {	
						// Delete it.
						deleteDir(runDir);
						System.out.println("Deleted old run directory " + path.toString());
					}
					
				} catch (IOException e) {
					e.printStackTrace();
				} catch (ParseException e) {
					e.printStackTrace();
				} 
			}
		}

	}

	// Returns true if runDataDir was successfully created.
	public static boolean instrumentationAvailable () {return instrAvailable;}
	
	// Returns the path to where debug data should be saved.
	public static String dataDirectoryName () {return runDataDir;}
	
	// Provides a common time source for child classes to time-tag debug data.
	// The returned time is in seconds.
	public static double timeNow () {
		return Timer.getFPGATimestamp();
	}

}
