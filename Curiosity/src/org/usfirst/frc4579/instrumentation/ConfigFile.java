/******************************************************************************
 * ConfigFile
 * 
 * This class allows for the creation and reading of configuration files that
 * contain parameters unique to a particular robot.  For example, one motor/
 * gearbox could have more friction than another, and the config file could
 * say how much more power needs to be provided to that motor in order to drive 
 * straight.
 * 
 * A config file is a simple text file can be created manually and copied to the 
 * roboRIO in the directory pointed to by the "user.home" property (the format of
 * the file is described in the writeConfigFileParameters method, below).  
 * Optionally, the roboRIO code can write the file itself via the 
 * writeConfigFileParameters method.  
 * 
 * A child class must provide a readConfigFileParameters method which will read
 * the parameters from the configuration file and make them visible via public 
 * member variables or public access methods.  
 * 
 * The parameters must be read from the file in the same order that they exist in
 * the file.
 ******************************************************************************/

package org.usfirst.frc4579.instrumentation;

import java.util.ArrayList;

public abstract class ConfigFile {
	
	private BasicTextFileOps cfgFile;
	
	/******************************************************************************
	 * Constructor
	 * 
	 * Recommend that the fileName be the name of the instantiating class, further
	 * suggesting that the name of that class should be representative of the content
	 * of the file.
	 ******************************************************************************/
	public ConfigFile (String fileName) {
		
		// String shortClassName = className.substring(className.indexOf('.')+1).trim();
		
		cfgFile = new BasicTextFileOps(System.getProperty("user.home") + "\\" + fileName + ".txt");
		
		// System.out.println("Class Name =" + fileName);
		
	}
	
	/******************************************************************************
	 * This method "searches" the file for a parameter associated with "key" and
	 * returns the value as a string.  The next non-comment line read must contain
	 * the "key" or an exception is raised.  This forces that values must be
	 * read from the file in the same order that they exist in the file.
	 ******************************************************************************/
	private String findValue (String key) {

		// if the file hasn't been opened yet, open it.
		if (!cfgFile.isOpenForRead()) cfgFile.openForRead();
		
		String str = "/";
		
		// Skip over any comment lines.
		while (str.startsWith("/")) str = cfgFile.readLine();

		// If the key doesn't match the what was read, throw an exception.
		if (!str.toUpperCase().startsWith(key.toUpperCase())) 
			throw new RuntimeException("Found " + str.toUpperCase().substring(0, str.indexOf(':')-1) + 
									   " when expecting " + key.toUpperCase() + ".");
		
		// Return a string that contains just the value to be read with no white space.
		return str.substring(str.indexOf(':')+1).trim();

	}
	
	/******************************************************************************
	 * The following methods may be used by the child class to read parameters
	 * from the configuration file.
	 ******************************************************************************/	
	protected double readDouble (String key) {
        return Double.parseDouble(findValue(key));
	}
	
	protected int readInt (String key) {
        return Integer.parseInt(findValue(key));
	}

	protected boolean readBoolean (String key) {
        return Boolean.parseBoolean(findValue(key));
	}
	
	/******************************************************************************
	 * The following method writes an arrayList of configuration file parameters to 
	 * the a configuration file. 
	 * 
	 * Example child ConfigFile child class:
	 * 
	 *    public class TestConfigFile extends ConfigFile {
	 *
	 *		  // Define config file parameters as public variables with default values.
	 *        public double  minDriveSpeed         = 0.1
	 *        public int     maxFluxCapacitorVolts = 100000;
	 *
	 *		  // Class name used as the file name.
	 *        public TestConfigFile() {
	 *  	      super(TestConfig.class.getName());
	 *        }
	 *
	 *		  // Implementation of abstract method that reads the parameters in the order
	 *		  // that they exist in the text file.
	 *        public void readConfigFileParameters() {
     *		      minDriveSpeed         = readDouble("Min drive speed");
	 *			  maxFluxCapacitorVolts = readInt   ("Max flux capacitor voltage");
	 *		  }
	 *	  }
	 *
	 * Example code that creates the TestConfigFile file:
	 * 
	 *    public TestConfigFile testConfigFile = new TestConfigFile();
	 *    
	 *    ArrayList<String> fileParams = new ArrayList<String>();
	 * 
	 *    fileParams.add("Min drive speed            : 0.5");
	 *    fileParams.add("Max flux capacitor voltage : 10000");
	 * 
	 *    testConfigFile.writeConfigFileParameters(fileParams);
	 *    
	 ******************************************************************************/
	public void writeConfigFileParameters(ArrayList<String> params) {
		
		cfgFile.openForWrite();
		
		cfgFile.writeLine("/ This file was auto-generated.  Comment lines begin with a '/'");
		cfgFile.writeLine("/ Each non-comment line must be of the form:  <key name> : <value> ");
		cfgFile.writeLine("/ Example:  'Min drive speed : 0.5'");
		cfgFile.writeLine("/ Key names are not case-sensitive.  The class that reads the parameters must");
		cfgFile.writeLine("/ read them in the same order that they were written, using the same key.");
		cfgFile.writeLine("/");
		
		for (String str : params)
			cfgFile.writeLine(str);
		
		cfgFile.close();
	}
	
	/******************************************************************************
	 * The following method must be implemented by the child class to read the
	 * parameters from the configuration file and make them available to consumers.
	 * It can be called from Robot.robotInit().
	 ******************************************************************************/
	public abstract void readConfigFileParameters();
	
}
