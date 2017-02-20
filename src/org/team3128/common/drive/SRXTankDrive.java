package org.team3128.common.drive;

import org.team3128.common.autonomous.AutoUtils;
import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;
import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Angle;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Class which represents a tank drive powered by Talon SRXs on a robot.
 * 
 * Uses positional PID for high-accuracy autonomous moves.
 * Make sure that the SRXs have quadrature encoders attached, have FeedbackDevice set, 
 * and that are configured to the correct number of counts per revolution.
 * 
 * 
 * @author Jamie
 *
 */
public class SRXTankDrive implements ITankDrive
{
	private CANTalon leftMotors;
    	
    private CANTalon rightMotors;
    
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
     */
    public SRXTankDrive(CANTalon leftMotors, CANTalon rightMotors, double wheelCircumfrence, double gearRatio, double wheelBase, double track)
    {
    	this.leftMotors = leftMotors;
    	this.rightMotors = rightMotors;
    	
    	this.wheelCircumfrence = wheelCircumfrence;
    	this.wheelBase = wheelBase;
    	this.track = track;
    	this.gearRatio = gearRatio;
    	
    	double turningCircleDiameter = Math.sqrt(RobotMath.square(track) + RobotMath.square(wheelBase)); //pythagorean theorem
    	turningCircleCircumference = turningCircleDiameter * Math.PI;
    	
    	if(gearRatio <= 0)
    	{
    		throw new IllegalArgumentException("Invalid gear ratio");
    	}
    	
    	setReversed(false);
    }

    private void configureForTeleop()
    {
    	if(!configuredForTeleop)
    	{
	    	leftMotors.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	    	rightMotors.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	    	
	    	configuredForTeleop = true;
    	}
    }
    
    private void configureForAuto()
    {
    	if(configuredForTeleop)
    	{
	    	leftMotors.changeControlMode(CANTalon.TalonControlMode.Position);
	    	rightMotors.changeControlMode(CANTalon.TalonControlMode.Position);
	    	
	    	configuredForTeleop = false;
    	}
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
	 * @param wheelCircumference the circumference of the wheels
	 * @return
	 */
	double cmToEncDegrees(double cm)
	{
		return (cm * 360) / (wheelCircumfrence * gearRatio);
	}
	
	/**
	 * Convert cm of robot movement to encoder rotations
	 * @param cm
	 * @param wheelCircumference the circumference of the wheels
	 * @return
	 */
	double encDistanceToCm(double encDistance)
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
    	
    	protected MoveEndMode endMode;
    	
    	boolean leftDone;
    	boolean rightDone;
    	
    	/**
    	 * @param leftDist Degrees to spin the left wheel
    	 * @param rightDist Degrees to spin the right wheel
    	 * @param power motor power to move at, from 0 to 1
    	 */
        public CmdMoveDistance(MoveEndMode endMode, double leftDist, double rightDist, double power, double timeout)
        {
        	super(timeout);
        	
        	this.power = power;
        	this.leftDist = leftDist;
        	this.rightDist = rightDist;
        	this.endMode = endMode;
        }

        protected void initialize()
        {
        	Log.info("CmdMoveDistance", "Initializing");
        	configureForAuto();
    		clearEncoders();
    		
    		leftMotors.set(leftDist / Angle.ROTATIONS);
    		rightMotors.set(rightDist / Angle.ROTATIONS);
    		
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
        	Log.debug("CmdMoveDistance", "left pos: " + leftMotors.getPosition() + " cle: " + leftMotors.getClosedLoopError() +
        			", right pos: " + rightMotors.getPosition() + " cle: " + rightMotors.getClosedLoopError());

        	leftDone = leftDist == 0  || RobotMath.abs((leftMotors.getClosedLoopError() / 1024.0) * Angle.ROTATIONS) < MOVEMENT_ERROR_THRESHOLD;
        	rightDone = rightDist == 0  || RobotMath.abs((rightMotors.getClosedLoopError()/ 1024.0) * Angle.ROTATIONS) < MOVEMENT_ERROR_THRESHOLD;
        	
        	switch(endMode)
        	{
        	case BOTH:
        		return leftDone && rightDone;
        	case EITHER:
        	default:
        		return leftDone || rightDone;
        	}
        }

        // Called once after isFinished returns true
        protected void end()
        {
        	Log.info("CmdMoveDistance", "Ending");

        	stopMovement();
        	if(isTimedOut())
        	{
    			AutoUtils.killRobot("Autonomous Move Overtime");
        	}
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
        public CmdArcTurn(float degs, int msec, double power, Direction dir)
        {
        	super(MoveEndMode.BOTH, 0, 0, power, msec);
        	
        	//this formula is explained on the info repository wiki
        	double wheelAngularDist = (2 * Math.PI * track) * (degs / 360);
        	
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
        	super(MoveEndMode.BOTH, 0, 0, motorPower, msec);
        	
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
        	super(MoveEndMode.BOTH, cmToEncDegrees(d), cmToEncDegrees(d), fullSpeed ? 1 : .50, msec);
        }
        
    	/**
    	 * @param d how far to move.  Accepts negative values.
    	 * @param msec How long the move should take. If set to 0, do not time the move
    	 */
        public CmdMoveForward(double d, int msec, double power)
        {
        	super(MoveEndMode.BOTH, cmToEncDegrees(d), cmToEncDegrees(d), power, msec);

        }
        
        @Override
        public void end()
        {
        	super.end();
        }
    }

   
}