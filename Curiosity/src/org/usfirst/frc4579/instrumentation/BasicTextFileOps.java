/******************************************************************************
 * BasicTextFileOps
 * 
 * This class provides basic services for reading and writing text files.
 * The following conventions are enforced:
 * 
 * 1. To read a text file:
 * 
 * 		myFile.openForRead();
 * 		str = myFile.readLine(); // Read one or more lines from the file.
 * 		...
 * 		myFile.close();
 * 
 * 2. To write a text file:
 * 
 * 		myFile.openForWrite();
 * 		myFile.readLine(str); // Write one or more lines to the file.
 * 		...
 * 		myFile.close();
 * 
 * If these conventions are violated this class throws a runtime exception.
 *****************************************************************************/

package org.usfirst.frc4579.instrumentation;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

public class BasicTextFileOps {

	// Data common to reads and writes.
	protected File	  file;
	protected String  fileName;
	
	// Data for reads
	private boolean        isOpenForRead = false;
    private BufferedReader br;
    
    // Data for writes.
	private boolean          isOpenForWrite = false;
	private BufferedWriter   bw;
	private FileWriter       fw;
    
	/******************************************************************************
	* Constructor
	******************************************************************************/
	public BasicTextFileOps (String fileName) {
		
		this.fileName = fileName;
		
		file = new File(fileName);
	}

	/******************************************************************************
	* Opens a file for reading.
	******************************************************************************/
	public void openForRead() {
		
		// Enforce that file is not already open.
		if (isOpenForRead)  throw new RuntimeException("ERROR: " + this.fileName + " is already open for read.");

		if (isOpenForWrite) throw new RuntimeException("ERROR: " + this.fileName + " is open for write.");

		// Enforce that the file must exist.
		if (!file.exists()) throw new RuntimeException("ERROR: " + this.fileName + " does not exist to read.");
		
		// Created the BufferedReader object.
		try {
			br = new BufferedReader(new FileReader(fileName));
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
		
		// Note that the file is open for reading.
		isOpenForRead = true;
		
	}
	
	/******************************************************************************
	* Returns "true" if the file is open for reading.
	******************************************************************************/
	public boolean isOpenForRead() {
		return this.isOpenForRead;
	}
	
	/******************************************************************************
	* Opens a file for writing.  If it already exists, it is deleted and re-
	* created.
	******************************************************************************/
	public void openForWrite() {

		// Enforce that file is not already open.
		if (isOpenForWrite) throw new RuntimeException("ERROR: " + this.fileName + " is already open for write.");

		if (isOpenForRead ) throw new RuntimeException("ERROR: " + this.fileName + " is already open for read.");
		
		// if the file already exists, delete it.
		if (file.exists()) delete();
		
		// if the file does not exist, try to create it.
		if (!file.exists()) {
			
			try {
				file.createNewFile();
			}
			catch (IOException e) {
				System.out.println("ERROR: File " + fileName + " could not be created.  Make sure that path name exists.");
				e.printStackTrace();
			}
		}

		try {
			fw = new FileWriter(file);
			bw = new BufferedWriter(fw);
		} catch (IOException e) {
			e.printStackTrace();
			return;
		}

		isOpenForWrite = true;

	}
	
	/******************************************************************************
	* Read a line from the file.  A null string is returned if at the end of file.
	******************************************************************************/
	public String readLine() {
		
		// Enforce that file is open for read.
		if (!isOpenForRead) throw new RuntimeException("ERROR: " + this.fileName + " is not open for read.");
		
		String str = null;
		
		try {
			str = br.readLine();
		}
	    catch (IOException e) {
		    e.printStackTrace();
	    }
		
		return str;
	}
	
	/******************************************************************************
	* Writes a line of text to the file.
	******************************************************************************/
	public void writeLine(String str) {
		
		// Enforce that file must be open for write.
		if (!isOpenForWrite) throw new RuntimeException("ERROR: " + this.fileName + " is not open for write.");
		
		try {
			bw.write(str + "\r\n");
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	/******************************************************************************
	* Closes an open file.
	******************************************************************************/
	public void close() {

		// Enforce that file is open.
		if (!isOpenForRead && !isOpenForWrite) throw new RuntimeException("ERROR: " + this.fileName + " is not open.");
	
		if (isOpenForRead) {
			try {
				br.close();
			} catch (IOException e) {
				e.printStackTrace();
			}

			isOpenForRead = false;
		}
		else {
			try {
				bw.close();
				fw.close();
			} catch (IOException e) {
				e.printStackTrace();
			}

			isOpenForWrite = false;
		}
	}
	
	/******************************************************************************
	* Deletes the file.
	******************************************************************************/
	public void delete() {

		// Enforce that file is open.
		if (isOpenForRead || isOpenForWrite) 
			throw new RuntimeException("ERROR: " + this.fileName + "  must be closed before deleting..");
	
		file.delete();
	}

}
