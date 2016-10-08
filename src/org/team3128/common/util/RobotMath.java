package org.team3128.common.util;

import org.team3128.common.util.enums.MotorDir;
import org.team3128.common.util.units.Angle;


/**
*
* @author Noah Sutton-Smolin
*/
public class RobotMath {
   /**
    * Limits the angle to between 0 and 359 degrees for all math. All angles
    * should be normalized before use.
    * <p/>
    * @param angle the angle to be normalized
    * <p/>
    * @return the normalized angle on [0, 359]
    */
   public static double normalizeAngle(double angle)
   {
       double theta = ((angle % 360) + 360) % 360;
       return theta;
   }

   //TODO: many of these functions exist in Java's Math class now that we aren't using Java ME
   /**
    * Finds the shortest distance between two angles.
    *
    * @param angle1 angle
    * @param angle2 angle
    * @param shortWay if true, go the shorter way to make moves always <= 180
    * @return shortest angular distance between
    */
   public static double angleDistance(double angle1, double angle2, boolean shortWay)
   {
       double dist = normalizeAngle(angle2) - normalizeAngle(angle1);
       
       if(shortWay && Math.abs(dist) > 180)
       {
           double sgn = RobotMath.sgn(dist);
           return -sgn * (360 - Math.abs(dist));
       }
       
       return dist;
   }

   
   /**
    * Standard-ish sign function
    * @param n
    * @return
    */
   public static double sgn(double n)
   {
	   if(n == 0)
	   {
		   return 0;
	   }
	   
       return Math.abs(n) / n;
   
   }
   
   public static int sgn(int n)
   {
	   if(n == 0)
	   {
		   return 0;
	   }
	   
       return Math.abs(n) / n;
   }
  

   /**
    * Determines the appropriate direction for a motor to turn to get to an angle.
    * <p/>
    * @param currentAngle the current angle of the motor
    * @param targetAngle the target angle of the motor
    * @param shortWay if true, go the shorter way to make moves always <= 180
    * <p/>
    * @return a MotorDir
    */
   public static MotorDir getMotorDirToTarget(double currentAngle, double targetAngle, boolean shortWay) 
   {
       currentAngle = RobotMath.normalizeAngle(currentAngle);
       targetAngle = RobotMath.normalizeAngle(targetAngle);
       int retDir = 1 * ((shortWay && Math.abs(currentAngle - targetAngle) > 180 )? 1 : -1) * (currentAngle - targetAngle < 0 ? -1 : 1);

       if (Math.abs(currentAngle - targetAngle) < .001) return MotorDir.NONE;
       return (retDir == 1 ? MotorDir.CW : MotorDir.CCW);
   }

   /**
    * Clamps value from (inclusive) minimum to maximum 
    * @param value
    * @param minimum
    * @param maximum
    * @return
    */
   public static int clamp(int value, int minimum, int maximum)
   {
	   if(!(minimum <= maximum))
	   {
		   Log.debug("RobotMath", "...what?  clampInt() called with insane arguments");
	   }
	   return Math.min(Math.max(value, minimum), maximum); 
   }
   
   /**
    * Clamps value from (inclusive) minimum to maximum 
    * @param value
    * @param minimum
    * @param maximum
    * @return
    */
   public static double clamp(double value, double minimum, double maximum)
   {
	   return Math.min(Math.max(value, minimum), maximum); 
   }
   
	/**
	 * Clamps value between positive and negative 1 and returns value.
	 * 
	 * Great for motor powers!
	 * @param d
	 */
	public static double clampPosNeg1(double d) {
		clamp(d, -1, 1);
		return d;
	}
   
   public static final double SQUARE_ROOT_TWO = Math.sqrt(2.0);
   
	/**
	 * If the abs value of the number is less than the threshold, return 0, otherwise return the number
	 * @param value
	 * @param threshold
	 * @return
	 */
	public static double thresh(double value, double threshold)
	{
		if(Math.abs(value) < Math.abs(threshold))
		{
			return 0;
		}
		return value;
	}
	
	/**
	 * Converts angular distance to linear.
	 * 
	 * For example, if a wheel was touching a surface, and the wheel turned y degrees, then the surface moved AngularDistToLinear(y) degrees.
	 * @param cm
	 * @param wheelCircumference the circumference of the circle
	 * @return
	 */
	public static double angularDistToLinear(double deg, double circumference)
	{
		return (deg / 360) * circumference;
	}
	

   //hidden constructor
   private RobotMath() {}
	
	
	/**
	 * Squares the argument.  Easier than Math.pow(number, 2).
	 * @param number
	 * @return
	 */
	public static double square(double number)
	{
		return number * number;
	}
	
	/**
	 * Squares the argument.  Easier than RobotMath.floor_double_int(Math.pow(number, 2)).
	 * @param number
	 * @return
	 */
	public static int square(int number)
	{
		return number * number;
	}
	
	/**
	 * Returns the exponent needed to make e (Euler's number) the given number.
	 * @param d
	 */
	public static double logE(double d) {
		return Math.log(d);
	}
	
	/**
	 * Returns the exponent needed to make 10 the given number.
	 * @param d
	 */
	public static double log10(double d) {
		return Math.log10(d);
	}
	
	//calculate this once here for speed
	static final private double log2Base10 = Math.log10(2);
	
	/**
	 * Returns the exponent needed to make 2 the given number.
	 * @param d
	 */
	public static double log2(double d) {
		return Math.log10(d) / log2Base10;
	}
	
	/**
	 * Returns the exponent needed to make an arbitrary base the given number.
	 * @param d
	 */
	public static double logN(double base, double num) {
		return Math.log(num) / Math.log(base);
	}
	
	/**
	 * Raises an integer to a integer power >= 0.
	 * 
	 * Much more lightweight than the regular function, but also more restricted.  Negative powers are treated as 0.
	 */
	public static int intPow(int number, int power)
	{
		int result = 1;
		for(; power >= 1; --power)
		{
			result *= number;
		}
		
		return result;
	}
	
	/**
	 * Raises a double to a integer power >= 0.
	 * 
	 * More lightweight than the regular function, but also more restricted.  Negative powers are treated as 0.
	 */
	public static double intPow(double number, int power)
	{
		double result = 1;
		for(; power >= 1; --power)
		{
			result *= number;
		}
		
		return result;
	}
	
	/**
	 * Returns d rounded to the nearest integer (ties round towards positive infinity).
	 * 
	 * Throws if the argument can't fit in an int.
	 * @param d
	 */
	public static int round(double d)
	{
		long roundedLong = Math.round(d);
		
		if(Math.abs(roundedLong) > Integer.MAX_VALUE)
		{
			throw new IllegalArgumentException("Provided number is too large to be an integer!");
		}
		
		return (int)roundedLong;
	}
	
	/**
	 * Returns the cosine of angle d in degrees.
	 * @param d
	 */
	public static double cos(double d) {
		return Math.cos(d / Angle.RADIANS); // convert to radians
	}
	
	/**
	 * Returns the sine of angle d in degrees.
	 * @param d
	 */
	public static double sin(double d) {
		return Math.sin(d / Angle.RADIANS);
	}
	
	/**
	 * Returns the tangent of angle d in degrees.
	 * @param d
	 */
	public static double tan(double d) {
		return Math.tan(d / Angle.RADIANS);
	}
	
	/**
	 * Returns the smallest integer greater to or equal to d.
	 * 
	 * Throws if the result cannot be represented by an integer.
	 * @param d
	 */
	public static int ceil(double d) 
	{
		double ceilinged = Math.ceil(d);
		if(!Double.isFinite(ceilinged) || ceilinged > Integer.MAX_VALUE)
		{
			throw new IllegalArgumentException("The provided double cannot be represented by an int");
		}
		
		return (int)ceilinged;
	}
	
	/**
	 * Returns the largest integer smaller to or equal to d.
	 * 
	 * Throws if the result cannot be represented by an integer.
	 * @param d
	 */
	public static int floor(double d)
	{
		double floored = Math.floor(d);
		   
		if(!Double.isFinite(floored) || floored > Integer.MAX_VALUE)
		{
			throw new IllegalArgumentException("The provided double cannot be represented by an int");
		}
		   
		return (int)floored;
	}
	
	/**
	 * Returns the arc-sine of d - the angle in degrees whose sine is d.
	 * @param d
	 */
	public static double asin(double d) {
		return Math.asin(d) * Angle.RADIANS;
	}
	
	/**
	 * Returns the arc-cosine of d - the angle in degrees whose cosine is d.
	 * @param d
	 */
	public static double acos(double d) {
		return Math.acos(d) * Angle.RADIANS;
	}
	
	
	/**
	 * Returns the arc-tangent of d - the angle in degrees whose tangent is d.
	 * @param d
	 */
	public static double atan(double d) {
		return Math.atan(d) * Angle.RADIANS;
	}
	
	/**
	 * Returns the angle in degrees whose Tan is y/x.
	 * 
	 * Takes the X and Y values separately so that the result can be placed in the correct quadrant.
	 * @param x
	 * @param y
	 */
	public static double atan2(double x, double y) {
		return Math.atan2(y, x) * Angle.RADIANS;
	}
	
	/**
	 * Returns the absolute value of an integer
	 * @param number
	 * @return
	 */
	public static int abs(int number)
	{
		return number < 0 ? -number : number;
	}
	
	/**
	 * Returns the absolute value of a double
	 * @param number
	 * @return
	 */
	public static double abs(double number)
	{
		return number < 0 ? -number : number;
	}

}
