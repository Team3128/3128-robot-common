package org.team3128.common.util;

import org.team3128.common.util.datatypes.PIDConstants;
import org.team3128.common.util.datatypes.Pair;
import org.team3128.common.util.datatypes.RandomAccessBuffer;

/**
 * Class to calculate positional PID
 * @author Jamie
 *
 */
public class PIDCalculator
{
	// if the time between updates exceeds this value, the integral will be reset
	private final double INTEGRATION_TIMEOUT = 1000; //ms
	
	private PIDConstants constants;
	
	// buffer containing the time duration that the feedback was a value, and the value
	private RandomAccessBuffer<Pair<Integer, Double>> pastValues;
	
	private double previousValue = 0;
	private long previousValueTime;
	
	private double target;
	
	/**
	 * @param izone th number of past measurements to keep when calculating the I term
	 */
	public PIDCalculator(PIDConstants constants, int izone)
	{
		this.constants = constants;
		pastValues = new RandomAccessBuffer<>(izone);
		previousValueTime = System.currentTimeMillis();
	}
	
	public void setConstants(PIDConstants constants)
	{
		this.constants = constants;
	}
	
	/**
	 * Zero out the accumulated I value
	 */
	public void resetIntegral()
	{
		for(int index= 0; index <= pastValues.getLastIndex(); ++index)
		{
			pastValues.set(index, null);
		}
	}
	
	public void setTarget(double target)
	{
		this.target = target;
	}
	
	/**
	 * Calculate the trapezoidal sum of the integration data.
	 * @return
	 */
	private double calculateIntegral()
	{
		if(pastValues.getSize() == 0)
		{
			return 0;
		}
		
		//we use trapezoidal integration
		
		double integral = 0;
		for(int index = 1; index <= pastValues.getLastIndex(); ++index)
		{
			integral += (pastValues.get(index - 1).right + pastValues.get(index).right) - pastValues.get(index).left;
		}
		integral /= 2;
		
		return integral;
	}
	
	public void update(double value)
	{
		// Cap the delta time to prevent it going through the roof when the robot is disabled
		int deltaTime = (int) RobotMath.clamp(System.currentTimeMillis() - previousValueTime, 0, 100);
		
		double error = target - value;
		
		double output = error * constants.kP;
		
	}
}
