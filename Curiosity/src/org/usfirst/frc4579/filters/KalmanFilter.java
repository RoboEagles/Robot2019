package org.usfirst.frc4579.filters;

public class KalmanFilter {

	private double Q_angle;   // Process noise variance for the accelerometer.
	private double Q_bias;    // Process noise variance for the gyro bias.
	private double R_measure; // Measurement noise variance.
	private double angle;     // The angle calculated by the Kalman filter.
	private double bias;      // The gyro bias calculated by the Kalman filter.
	private double rate;
	private double[][] P;     // Error covariance matrix.
	private double[] K;       // Kalman gain 2x1 vector.
	private double y;         // Angle difference.
	private double S;         // Estimate error.
	
	public KalmanFilter () {
		
		Q_angle   = 0.001;
		Q_bias    = 0.003;
		R_measure = 0.03;
		angle     = 0.0;
		bias      = 0.0;
		P         = new double[2][2];
		K         = new double[2];
		P[0][0]   = 0.0; // Since we assume that the bias is 0 and we know the starting angle (use setAngle),
		P[0][1]   = 0.0; // the error covariance matrix is set like so.
		P[1][0]   = 0.0;
		P[1][1]   = 0.0;
	}
	
	// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds.
	public double getAngle (double newAngle, double newRate, double dt) {
		
		// Project the state ahead (xhat)
		rate   = newRate - bias;
		angle += dt * rate;
		
		// Update the estimation error covariance.
		P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
		P[0][1] -= dt * P[1][1];
		P[1][0] -= dt * P[1][1];
		P[1][1] += dt * Q_bias;
		
		// Compute the Kalman gain.
		S    = P[0][0] + R_measure;
		K[0] = P[0][0] / S;
		K[1] = P[1][0] / S;
		
		// Calculate angle and bias - update estimate with measurement.
		y = newAngle - angle;
		angle += K[0] * y;
		bias  += K[1] * y;
		
		// Update the error covariance.
		P[0][0] -= K[0] * P[0][0];
		P[0][1] -= K[0] * P[0][1];
		P[1][0] -= K[1] * P[0][0];
		P[1][1] -= K[1] * P[0][1];
		
		return angle;
	}
	
	// Used to set the starting angle.
	public void setAngle(double newAngle) { angle = newAngle; }
	
	// Get the unbiased rate;
	double getRate() { return rate;}
	
	// These are used to tune the Kalman filter.
	public void setQangle   ( double newQ_angle)    { Q_angle = newQ_angle; }
	public void setQbias    ( double newQ_bias)     { Q_bias  = newQ_bias;  }
	public void setRmeasure ( double newR_measure ) { R_measure = newR_measure; }
	
	double getQangle   () { return Q_angle; }
	double getQbias    () { return Q_bias;  }
	double getRmeasure () { return R_measure; }
	
}
