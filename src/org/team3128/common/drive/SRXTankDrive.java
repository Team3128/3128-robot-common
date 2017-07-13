package org.team3128.common.drive;

import org.team3128.common.hardware.misc.TwoSpeedGearshift;
import org.team3128.common.util.Assert;
import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;
import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Angle;
import org.team3128.common.util.units.Length;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Class which represents a tank drive powered by Talon SRXs on a robot.
 * 
 * Uses positional PID for high-accuracy autonomous moves.
 * Make sure that the SRXs have quadrature encoders attached, have FeedbackDevice set, 
 * and that are configured to the correct number of counts per revolution.
 * 
 * CALIBRATION PROCEDURE:
 * -------------------------------------------------------------------------------------------------
 * Get the PID constants to ballpark.
 * EACH DAY OF EACH COMPETITION:
 * - Run the drive forward on the practice field for 100 inches at the speed that is used for the competition autos
 *    - Adjust feedforward on each side until the robot drives straight
 *    - Adjust the wheel diameter until the distance is correct to the inch
 * @author Jamie
 *
 */
public class SRXTankDrive implements ITankDrive
{
	private CANTalon leftMotors, rightMotors;
    
    private TwoSpeedGearshift gearshift;
    
    /**
     * The minimum speed (in RPM) of the wheels at which the robot should shift up to high gear if the robot was previously in low gear
     */
    private double shiftUpSpeed;
    
    /**
     * The maximum speed (in RPM) of the wheels at which the robot should shift down to low gear if the robot was previously in high gear
     */
    private double shiftDownSpeed;
    
    /**
     * True if the talons are set in PercentVbus mode for teleop driving, false if they are in position PID mode for auto.
     */
    private boolean configuredForTeleop;
	
    
    /**
     * circumference of wheels in cm
     */
    public final double wheelCircumfrence;
    
    /**
     * horizontal distance between wheels in cm
     */
    public final double wheelBase;
    
    /**
     * distance between front and back wheels
     */
    public final double track;
    
    /**
     * Circumference of the turning circle when in-place turning
     */
    public final double turningCircleCircumference;
    
    /**
     * Ratio between turns of the wheels to turns of the encoder
     */
    private double gearRatio;
    
    /**
     * Inversions for each side of the drive.
     */
    private boolean leftInverted, rightInverted;
    
    /**
     * Measured free speed of the robot at 100% throttle
     */
    private double robotMaxSpeed;
    
    /**
     * Speed scalar for the left and right wheels.  Affects autonomous and teleop.
     */
    private double leftSpeedScalar, rightSpeedScalar;
    
    
    public double getGearRatio()
	{
		return gearRatio;
	}

	public void setGearRatio(double gearRatio)
	{
		this.gearRatio = gearRatio;
	}

	/**
     * If there is more than one motor per side, configure each additional Talon to follow the one with the encoder using Follower mode.
     * 
     * @param leftMotors The "lead" Talon SRX on the left side.
     * @param rightMotors The "lead" Talon SRX on the right side.
     * @param wheelCircumfrence The circumference of the wheel
     * @param gearRatio The gear ratio of the turns of the wheels per turn of the encoder shaft
     * @param wheelBase The distance between the front and back wheel on a side
     * @param track distance across between left and right wheels
     * @param robotFreeSpeed the measured maximum speed (in RPM) of the robot when it is driving
     */
    public SRXTankDrive(CANTalon leftMotors, CANTalon rightMotors, double wheelCircumfrence, double gearRatio, double wheelBase, double track, double robotFreeSpeed)
    {
    	this.leftMotors = leftMotors;
    	this.rightMotors = rightMotors;
    	
    	this.wheelCircumfrence = wheelCircumfrence;
    	this.wheelBase = wheelBase;
    	this.track = track;
    	this.gearRatio = gearRatio;
    	this.robotMaxSpeed = robotFreeSpeed;
    	
    	double turningCircleDiameter = Math.sqrt(RobotMath.square(track) + RobotMath.square(wheelBase)); //pythagorean theorem
    	turningCircleCircumference = turningCircleDiameter * Math.PI;
    	
    	leftSpeedScalar = 1;
    	rightSpeedScalar = 1;
    	
    	if(gearRatio <= 0)
    	{
    		throw new IllegalArgumentException("Invalid gear ratio");
    	}
    	
    	// act like battery voltage is always 11V if it is higher
    	this.leftMotors.setNominalClosedLoopVoltage(11.0);
    	this.rightMotors.setNominalClosedLoopVoltage(11.0);
    	
    	setReversed(false);
    }
    

    private void configureForTeleop()
    {
    	if(!configuredForTeleop)
    	{
	    	leftMotors.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	    	rightMotors.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	    	
	    	leftMotors.enableBrakeMode(true);
	    	rightMotors.enableBrakeMode(true);
	    	
	    	configuredForTeleop = true;
    	}
    }
    
    private void configureForAuto()
    {
    	// autonomous commands may have changed this stuff, so we always set it
    	leftMotors.changeControlMode(CANTalon.TalonControlMode.MotionMagic);
    	rightMotors.changeControlMode(CANTalon.TalonControlMode.MotionMagic);
    	
    	leftMotors.enableBrakeMode(true);
    	rightMotors.enableBrakeMode(true);

    	configuredForTeleop = false;
    }
    
	//threshold below which joystick movements are ignored.
	final static double thresh = 0.2;
	
	
	/**
	 * Update the motor outputs with the given control values.
	 * @param joyX horizontal control input
	 * @param joyY vertical control input
	 * @param throttle throttle control input scaled between 1 and -1 (-.8 is 10 %, 0 is 50%, 1.0 is 100%)
	 */
	@Override
    public void arcadeDrive(double joyX, double joyY, double throttle, boolean fullSpeed)
    {
    	configureForTeleop();
    	
        double spdL, spdR;
    	
    	if(!fullSpeed)
    	{
    		joyY *= .65;
    	}
    	else
    	{
    		joyY *= 1;
    	}
    	
    	//scale from 1 to -1 to 1 to 0
    	throttle =  ( throttle + 1) / 2;

    	if(throttle < .3)
    	{
    		throttle = .3;
    	}
    	else if(throttle > .8)
    	{
    		throttle = 1;
    	}
    	
    	joyY *= throttle;
    	joyX *= throttle;
    	
    	spdR = RobotMath.clampPosNeg1(joyY + joyX);
    	spdL = RobotMath.clampPosNeg1(joyY - joyX);
    	
    	if(leftInverted)
    	{
    		spdL *= -1;
    	}
    	
    	if(rightInverted)
    	{
    		spdR *= -1;
    	}
    	
    	Log.debug("SRXTankDrive", "x1: " + joyX + " throttle: " + throttle + " spdR: " + spdR + " spdL: " + spdL);

    	leftMotors.set(spdL);
    	rightMotors.set(spdR);
    }
	
	/**
	 * Set the left speed scalar.  Must be between 0 and 1.
	 */
	public void setLeftSpeedScalar(double scalar)
	{
		Assert.inRange(scalar, 0, 1);
		leftSpeedScalar = scalar;
	}
	
	/**
	 * Set the right speed scalar.  Must be between 0 and 1.
	 */
	public void setRightSpeedScalar(double scalar)
	{
		Assert.inRange(scalar, 0, 1);
		rightSpeedScalar = scalar;
	}
	
	/**
	 * Set whether the drive is reversed (switch the side that is inverted).
	 * By default, the right motors are inverted, and the left are not.
	 * 
	 */
	public void setReversed(boolean reversed)
	{
		leftInverted = reversed;
		rightInverted = !reversed;
		
		// this affects closed-loop control only
		rightMotors.reverseOutput(!reversed);
		leftMotors.reverseOutput(reversed);
	}
	
	public void setReversedAutonomous(boolean reversed)
	{
		// this affects closed-loop control only
		rightMotors.reverseOutput(!reversed);
		leftMotors.reverseOutput(reversed);
	}
	
	public void setReversedTeleop(boolean reversed)
	{
		leftInverted = reversed;
		rightInverted = !reversed;
	}
    
    /**
     * Drive by providing motor powers for each side.
     * @param powL the left side power.
     * @param powR the right side power.
     */
    public void tankDrive(double powL, double powR)
    {
    	configureForTeleop();
    	leftMotors.set((leftInverted ? -1 : 1) * powL);
    	rightMotors.set((rightInverted ? -1 : 1) * powR);
    }
    
	public void clearEncoders()
	{
		leftMotors.setPosition(0);
		rightMotors.setPosition(0);
	}

	@Override
	public void stopMovement()
	{
		// not sure about the best way to do this
		// we can disable the motors, but then we have to reenable them later
		// so I do it this way instead
		
		configureForTeleop();
		tankDrive(0, 0);
	}
	
	public void addShifter(TwoSpeedGearshift gearshift, double shiftUpSpeed, double shiftDownSpeed) {
		this.gearshift = gearshift;
		this.shiftUpSpeed = shiftUpSpeed;
		this.shiftDownSpeed = shiftDownSpeed;
	}
	
	public void shiftToHigh() {
		if (gearshift != null) {
			gearshift.shiftToHigh();
		}
		else {
			Log.info("SRXTankDrive", "You can't shift gears. The robot doesn't actually have a gearshift.");
		}
	}
	
	public void shiftToLow() {
		if (gearshift != null) {
			gearshift.shiftToLow();
		}
		else {
			Log.info("SRXTankDrive", "You can't shift gears. The robot doesn't actually have a gearshift.");
		}
	}
	
	public void shift() {
		if (gearshift != null) {
			gearshift.shiftToOtherGear();
		}
		else {
			Log.info("SRXTankDrive", "You can't shift gears. The robot doesn't actually have a gearshift.");
		}
	}
	
	public boolean isInHighGear() {
		if (gearshift != null) {
			return gearshift.isInHighGear();
		}
		else {
			Log.fatal("SRXTankDrive", "There is only one gear. The robot doesn't actually have a gearshift. The code that involves this is probably bad news.");
			return false;
		}
	}
	
	public void autoshift() {
		if (gearshift != null) {
			if ( (rightMotors.getSpeed() < 0 && leftMotors.getSpeed() > 0) || (rightMotors.getSpeed() > 0 && leftMotors.getSpeed() < 0) ) {
				gearshift.shiftToHigh();
			}
			else if (!gearshift.isInHighGear() && (rightMotors.getSpeed() > shiftUpSpeed || leftMotors.getSpeed() > shiftUpSpeed)) {
				gearshift.shiftToHigh();
			}
			else if (gearshift.isInHighGear() && (rightMotors.getSpeed() < shiftDownSpeed && leftMotors.getSpeed() < shiftDownSpeed)) {
				gearshift.shiftToLow();
			}
		}
		else {
			Log.info("SRXTankDrive", "You can't shift gears. The robot doesn't actually have a gearshift.");
		}
	}
	
	
	/**
	 * Get the estimated angle that the robot has turned since the encoders were last reset, based on the relative distances of each side.
	 * 
	 * Range: [0, 360)
	 * 0 degrees is straight ahead.
	 * @return
	 */
	public double getRobotAngle()
	{
		double leftDist = encDistanceToCm(leftMotors.getPosition() * Angle.ROTATIONS);
		double rightDist = encDistanceToCm(rightMotors.getPosition() * Angle.ROTATIONS);
		
		double difference = leftDist - rightDist;
		
		return RobotMath.normalizeAngle((difference / turningCircleCircumference) * Angle.ROTATIONS);
	}
	
	/**
	 * Convert cm of robot movement to encoder movement in degrees
	 * @param cm
	 * @return
	 */
	public double cmToEncDegrees(double cm)
	{
		return (cm * 360) / (wheelCircumfrence * gearRatio);
	}
	
	/**
	 * Convert cm of robot movement to encoder rotations
	 * @param cm
	 * @return
	 */
	public double encDistanceToCm(double encDistance)
	{
		return (encDistance / 360) * wheelCircumfrence * gearRatio;
	}
	
    /**
     * Enum for how CmdMoveDistance determines when to end a move command.
     * @author Jamie
     *
     */
	public enum MoveEndMode
	{
		BOTH, //ends when both sides have reached their targets.  
		EITHER, //Stops both sides when either side has reached its target.  Force stops the move command of the slower side.
	}
    
  
	/**
     * Command to move each side of the drivetrain a specified distance.
     * 
     * Common logic shared by all of the autonomous movement commands
     */
    public class CmdMoveDistance extends Command
    {
    	//when the wheels' angular distance get within this threshold of the correct value, that side is considered done
    	final static double MOVEMENT_ERROR_THRESHOLD = 10 * Angle.DEGREES; 
    	
    	protected double power;
    	
    	protected double leftDist, rightDist;
    	
    	protected int correctDistanceCount = 0;
    	
    	protected MoveEndMode endMode;
    	
    	boolean leftDone;
    	boolean rightDone;
    	
    	boolean useScalars;
    	
    	/**
    	 * @param leftDist Degrees to spin the left wheel
    	 * @param rightDist Degrees to spin the right wheel
    	 * @param power motor power to move at, from 0 to 1
    	 */
        public CmdMoveDistance(MoveEndMode endMode, double leftDist, double rightDist, double power, boolean useScalars, double timeout)
        {
        	super(timeout / 1000.0);
        	
        	this.power = power;
        	this.leftDist = leftDist;
        	this.rightDist = rightDist;
        	this.endMode = endMode;
        	this.useScalars = useScalars;
        }

        protected void initialize()
        {
        	Log.info("CmdMoveDistance", "Initializing");
        	configureForAuto();
    		clearEncoders();
    		
    		double leftSpeed = robotMaxSpeed * power * ((useScalars) ? leftSpeedScalar : 1.0);
    		double rightSpeed = robotMaxSpeed * power * ((useScalars) ? rightSpeedScalar : 1.0);
    		
    		if (useScalars)
    		{
    			Log.info("SRXTankDrive", "Using scalars.");
    		}
    		
    		// motion magic does not work well when the distance is 0
    		if(leftDist == 0)
    		{
    			leftMotors.changeControlMode(TalonControlMode.Position);
    		}
    		if(rightDist == 0)
    		{
    			rightMotors.changeControlMode(TalonControlMode.Position);
    		}

			leftMotors.setMotionMagicCruiseVelocity(leftSpeed);
    		leftMotors.setMotionMagicAcceleration(leftSpeed / 1.5);
    		
    		leftMotors.set(leftDist / Angle.ROTATIONS);

    		rightMotors.setMotionMagicCruiseVelocity(rightSpeed);
    		rightMotors.setMotionMagicAcceleration(rightSpeed / 1.5);
    		
    		rightMotors.set(rightDist / Angle.ROTATIONS);
		
    		
    		Log.debug("CmdMoveDistance", "Distances: L:" + leftDist/Angle.ROTATIONS + " rot, R: " + rightDist/Angle.ROTATIONS + " rot, Speeds: L: " + leftSpeed + " RPM, R: " + rightSpeed + " RPM");
    		
    		try
			{
				Thread.sleep(100);
			}
			catch (InterruptedException e)
			{
				e.printStackTrace();
			}
        }

        // Make this return true when this Command no longer needs to run execute()
        protected boolean isFinished()
        {
        	double leftError = leftMotors.getPosition() * Angle.ROTATIONS - leftDist;
        	double rightError = rightMotors.getPosition() * Angle.ROTATIONS - rightDist;
        	
        	Log.debug("CmdMoveDistance", "left pos: " + leftMotors.getPosition() + " err: " + leftError + "deg, right pos: " + rightMotors.getPosition() + " err: " + rightError);

        	leftDone = leftDist == 0  || RobotMath.abs(leftError) < MOVEMENT_ERROR_THRESHOLD;
        	rightDone = rightDist == 0  || RobotMath.abs(rightError) < MOVEMENT_ERROR_THRESHOLD;
        	
        	if(isTimedOut())
        	{
    			Log.unusual("CmdMoveDistance", "Autonomous Move Overtime");
    			return true;
        	}
        	
        	boolean isInZone;
        	
        	switch(endMode)
        	{
        	case BOTH:
        		isInZone = leftDone && rightDone;
        		break;
        	case EITHER:
        	default:
        		isInZone = leftDone || rightDone;
        		break;
        	}
        	
        	if(isInZone)
        	{
        		++correctDistanceCount;
        	}
        	else
        	{
        		correctDistanceCount = 0;
        	}
        	
        	return correctDistanceCount > 15;
        }

        // Called once after isFinished returns true
        protected void end()
        {
        	Log.info("CmdMoveDistance", "Ending");

    		if(leftDist == 0)
    		{
    			leftMotors.changeControlMode(TalonControlMode.MotionMagic);
    		}
    		if(rightDist == 0)
    		{
    			rightMotors.changeControlMode(TalonControlMode.MotionMagic);
    		}
    		
        	stopMovement();
        }

        // Called when another command which requires one or more of the same
        // subsystems is scheduled to run
        protected void interrupted()
        {
        	stopMovement();
        }

		@Override
		protected void execute()
		{
			//do nothing
		}
    }
	
    /**
     * Command to to an arc turn in the specified amount of degrees.
     * 
     * Runs the opposite motors from the direction provided, so turning LEFT would set the RIGHT motors.
     * 
     * NOTE: currently requires that the front or back wheels be omni wheels for accurate turning.
     */
    public class CmdArcTurn extends CmdMoveDistance {
    	
    	/**
    	 * @param degs how far to turn in degrees.  Accepts negative values.
    	 * @param msec How long the move should take. If set to 0, do not time the move.
    	 */
        public CmdArcTurn(float degs, int msec, Direction dir)
        {
        	this(degs, msec, dir, .5);
        }
        
        //it seems like the math is consistently off by about 6%
        final static double FUDGE_FACTOR = 1.06;
        
    	/**
    	 * @param degs how far to turn in degrees.  Accepts negative values.
    	 * @param msec How long the move should take. If set to 0, do not time the move.
    	 */
        public CmdArcTurn(float degs, int msec, Direction dir, double power)
        {
        	super(MoveEndMode.BOTH, 0, 0, power, false, msec);
        	
        	//this formula is explained on the info repository wiki
        	double wheelAngularDist = cmToEncDegrees((2 * Math.PI * track) * (degs / 360)) * FUDGE_FACTOR;
        	
        	if(dir == Direction.RIGHT)
        	{
        		leftDist = wheelAngularDist;
        	}
        	else
        	{
        		rightDist = wheelAngularDist;
        	}
        }
    }
    
    
    /**
     * Command to to an arc turn in the specified amount of degrees.
     * 
     * Sets the opposite motors from the direction provided, so turning LEFT would set the RIGHT motors.
     */
    public class CmdInPlaceTurn extends CmdMoveDistance 
    {
    	/**
    	 * @param degs how far to turn in degrees.  Accepts negative values.
    	 * @param msec How long the move should take. If set to 0, do not time the move
    	 */
        public CmdInPlaceTurn(float degs, int msec, Direction dir)
        {
        	this(degs, .5, msec, dir);
        }
        
        /**
    	 * @param degs how far to turn in degrees.  Accepts negative values.
    	 * @param msec How long the move should take. If set to 0, do not time the move
    	 */
        public CmdInPlaceTurn(float degs, double motorPower, int msec, Direction dir)
        {
        	//the encoder counts are an in-depth calculation, so we don't set them until after the super constructor
        	super(MoveEndMode.BOTH, 0, 0, motorPower, false, msec);
        	
        	//this formula is explained in the info repository wiki
    		double wheelAngularDist = cmToEncDegrees(turningCircleCircumference*(degs/360.0)); 
        	
        	if(dir == Direction.RIGHT)
        	{
        		leftDist = wheelAngularDist;
        		rightDist = -wheelAngularDist;
        	}
        	else
        	{
        		leftDist = -wheelAngularDist;
        		rightDist = wheelAngularDist;
        	}
        }
    }
    
    
    /**
     * Command to move forward the given amount of centimeters.  Drives straight, if you've set up your 
     * speed multipliers properly.
     */
    public class CmdMoveForward extends CmdMoveDistance 
    {
    	
    	/**
    	 * @param d how far to move.  Accepts negative values.
    	 * @param msec How long the move should take. If set to 0, do not time the move
    	 */
        public CmdMoveForward(double d, int msec, boolean fullSpeed)
        {
        	super(MoveEndMode.BOTH, cmToEncDegrees(d), cmToEncDegrees(d), fullSpeed ? 1 : .50, true, msec);
        }
        
    	/**
    	 * @param d how far to move.  Accepts negative values.
    	 * @param msec How long the move should take. If set to 0, do not time the move
    	 */
        public CmdMoveForward(double d, int msec, double power)
        {
        	super(MoveEndMode.BOTH, cmToEncDegrees(d), cmToEncDegrees(d), power, true, msec);

        }
        
        @Override
        public void end()
        {
        	super.end();
        }
    }

    public class CmdFeedForwardFineTune extends CmdMoveDistance
    {

		public CmdFeedForwardFineTune(double power, boolean useScalars, double timeout)
		{
			super(MoveEndMode.BOTH, 100 * Length.in, 100 * Length.in, power, false, timeout);
			// TODO Auto-generated constructor stub
		}
    	
    }
}