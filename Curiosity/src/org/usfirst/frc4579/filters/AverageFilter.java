package org.usfirst.frc4579.filters;

/*************************************************************************************
 * AVERAGE FILTER
 * 
 * Allows the caller to maintain a running average of sample values.  Standard deviation
 * is also available.  The class is instantiated with the max number of samples that
 * should be averaged.
 *
 *************************************************************************************/

public class AverageFilter {
	
	private final int arraySize;      // Number of elements in averageArray.
	private int       newest = -1;    // Array index of the newest entry in the array.
	private int       numEntries = 0; // Number of entries used in the array.
	private double    sum = 0.0;      // Current sum of array entries.
	private double[]  averageArray;   // The array that contains the sample values.
	
	//*************************************************************************************
	// Constructor.
	//*************************************************************************************
	public AverageFilter(int numberOfSamples) {
		this.arraySize    = numberOfSamples;
		this.averageArray = new double[numberOfSamples];		
	}
	
	//*************************************************************************************
	// Add "input" into the running average without returning the average (filtered value).
	//*************************************************************************************
	public void accumulate (double input) {
		
		// Add the new value into the running sum.
		sum += input;
		
		// Compute the array index of where to put the newest input.
		newest = (newest+1) % arraySize;
		
		// if the array is full
		if (numEntries == arraySize) {
			
			// "newest" has wrapped around and now points to the oldest entry. 
			// Remove the oldest entry from the sum.
			sum -= averageArray[newest];
			
		}
		else
			// Increment the number of samples in the array.
			numEntries++;

		// Save the newest value.
		averageArray[newest] = input;
	}
	
	//*************************************************************************************
	// Add "input" into the running average and return the average (filtered value).
	//*************************************************************************************
	public double filter(double input) {
		
		accumulate(input);

		return average();
	}
	
	//*************************************************************************************
	// Return the current running average.
	//*************************************************************************************
	public double average () { return sum / (double)numEntries; }
	
	//*************************************************************************************
	// Return the standard deviation of the current running average.
	//*************************************************************************************
	public double stdDeviation () { 
		
		double avg    = average();
		double stdDev = 0.0;
		
		for (int i=0; i < numEntries; i++) stdDev += Math.pow((averageArray[i] - avg), 2.0);
		
		stdDev = Math.sqrt(stdDev / (double)numEntries);
		
		return stdDev; 
	}

}
