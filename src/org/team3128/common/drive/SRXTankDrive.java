package org.team3128.common.drive;

import org.team3128.common.hardware.misc.TwoSpeedGearshift;
import org.team3128.common.util.Assert;
import org.team3128.common.util.Constants;
import org.team3128.common.util.Log;
import org.team3128.common.util.RobotMath;
import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Angle;
import org.team3128.common.util.units.AngularSpeed;
import org.team3128.common.util.units.Length;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Class which represents a tank drive powered by Talon SRXs on a robot.
 * 
 * Uses positional PID for high-accuracy autonomous moves. Make sure that the
 * SRXs have quadrature encoders attached, have FeedbackDevice set, and that are
 * configured to the correct number of counts per revolution.
 * 
 * CALIBRATION PROCEDURE:
 * -------------------------------------------------------------------------------------------------
 * Get the PID constants to ballpark. EACH DAY OF EACH COMPETITION: - Run the
 * drive forward on the practice field for 100 inches at the speed that is used
 * for the competition autos - Adjust feedforward on each side until the robot
 * drives straight - Adjust the wheel diameter until the distance is correct to
 * the inch
 * 
 * @author Jamie
 *
 */
public class SRXTankDrive implements ITankDrive
{
	private TalonSRX leftMotors, rightMotors;

	private TwoSpeedGearshift gearshift;

	/**
	 * The minimum speed (in RPM) of the wheels at which the robot should shift
	 * up to high gear if the robot was previously in low gear
	 */
	private double shiftUpSpeed;

	/**
	 * The maximum speed (in RPM) of the wheels at which the robot should shift
	 * down to low gear if the robot was previously in high gear
	 */
	private double shiftDownSpeed;

	/**
	 * True if the talons are set in PercentVbus mode for teleop driving, false
	 * if they are in position PID mode for auto.
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
	 * Measured free speed in native units per 100ms of the robot at 100%
	 * throttle
	 */
	private int robotMaxSpeed;

	/**
	 * Speed scalar for the left and right wheels. Affects autonomous and
	 * teleop.
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
	 * If there is more than one motor per side, configure each additional Talon
	 * to follow the one with the encoder using Follower mode.
	 * 
	 * @param leftMotors
	 *            The "lead" NarwhalSRX on the left side.
	 * @param rightMotors
	 *            The "lead" NarwhalSRX on the right side.
	 * @param wheelCircumfrence
	 *            The circumference of the wheel
	 * @param gearRatio
	 *            The gear ratio of the turns of the wheels per turn of the
	 *            encoder shaft
	 * @param wheelBase
	 *            The distance between the front and back wheel on a side
	 * @param track
	 *            distance across between left and right wheels
	 * @param robotFreeSpeed
	 *            the measured maximum speed in native units per 100ms of the
	 *            robot when it is driving in low gear
	 */
	public SRXTankDrive(TalonSRX leftMotors, TalonSRX rightMotors, double wheelCircumfrence, double gearRatio,
			double wheelBase, double track, int robotFreeSpeed)
	{
		this.leftMotors = leftMotors;
		this.rightMotors = rightMotors;

		this.wheelCircumfrence = wheelCircumfrence;
		this.wheelBase = wheelBase;
		this.track = track;
		this.gearRatio = gearRatio;
		this.robotMaxSpeed = robotFreeSpeed;

		double turningCircleDiameter = Math.sqrt(RobotMath.square(track) + RobotMath.square(wheelBase)); // pythagorean
																											// theorem
		turningCircleCircumference = turningCircleDiameter * Math.PI;

		leftSpeedScalar = 1;
		rightSpeedScalar = 1;

		if (gearRatio <= 0)
		{
			throw new IllegalArgumentException("Invalid gear ratio");
		}

		setReversed(false);
	}

	private void configureForTeleop()
	{
		if (!configuredForTeleop)
		{
			// Now encapsulated in the set() method as of CTRE Pheonix
			// leftMotors.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);
			// rightMotors.changeControlMode(TalonSRX.TalonControlMode.PercentVbus);

			leftMotors.setNeutralMode(NeutralMode.Brake);
			rightMotors.setNeutralMode(NeutralMode.Brake);

			configuredForTeleop = true;
		}
	}

	private void configureForAuto()
	{
		// Now encapsulated in the set() method as of CTRE Pheonix
		// autonomous commands may have changed this stuff, so we always set it
		// leftMotors.changeControlMode(TalonSRX.TalonControlMode.MotionMagic);
		// rightMotors.changeControlMode(TalonSRX.TalonControlMode.MotionMagic);

		leftMotors.setNeutralMode(NeutralMode.Brake);
		rightMotors.setNeutralMode(NeutralMode.Brake);

		configuredForTeleop = false;
	}

	// threshold below which joystick movements are ignored.
	final static double thresh = 0.2;

	/**
	 * Update the motor outputs with the given control values.
	 * 
	 * @param joyX
	 *            horizontal control input
	 * @param joyY
	 *            vertical control input
	 * @param throttle
	 *            throttle control input scaled between 1 and -1 (-.8 is 10 %, 0
	 *            is 50%, 1.0 is 100%)
	 */
	@Override
	public void arcadeDrive(double joyX, double joyY, double throttle, boolean fullSpeed)
	{
		configureForTeleop();

		double spdL, spdR;

		if (!fullSpeed)
		{
			joyY *= .65;
		}
		else
		{
			joyY *= 1;
		}

		// scale from 1 to -1 to 1 to 0
		throttle = (throttle + 1) / 2;

		if (throttle < .3)
		{
			throttle = .3;
		}
		else if (throttle > .8)
		{
			throttle = 1;
		}

		joyY *= throttle;
		joyX *= throttle;

		spdR = RobotMath.clampPosNeg1(joyY + joyX);
		spdL = RobotMath.clampPosNeg1(joyY - joyX);

		if (leftInverted)
		{
			spdL *= -1;
		}

		if (rightInverted)
		{
			spdR *= -1;
		}

		// Log.debug("SRXTankDrive", "x1: " + joyX + " throttle: " + throttle +
		// " spdR: " + spdR + " spdL: " + spdL);

		leftMotors.set(ControlMode.PercentOutput, spdL);
		rightMotors.set(ControlMode.PercentOutput, spdR);
	}

	/**
	 * Set the left speed scalar. Must be between 0 and 1.
	 */
	public void setLeftSpeedScalar(double scalar)
	{
		Assert.inRange(scalar, 0, 1);
		leftSpeedScalar = scalar;
	}

	/**
	 * Set the right speed scalar. Must be between 0 and 1.
	 */
	public void setRightSpeedScalar(double scalar)
	{
		Assert.inRange(scalar, 0, 1);
		rightSpeedScalar = scalar;
	}

	/**
	 * Set whether the drive is reversed (switch the side that is inverted). By
	 * default, the right motors are inverted, and the left are not.
	 * 
	 */
	public void setReversed(boolean reversed)
	{
		leftInverted = reversed;
		rightInverted = !reversed;

		// this affects closed-loop control only
		rightMotors.setInverted(!reversed);
		leftMotors.setInverted(reversed);
	}

	public void setReversedAutonomous(boolean reversed)
	{
		// this affects closed-loop control only
		rightMotors.setInverted(!reversed);
		leftMotors.setInverted(reversed);
	}

	public void setReversedTeleop(boolean reversed)
	{
		leftInverted = reversed;
		rightInverted = !reversed;
	}

	/**
	 * Drive by providing motor powers for each side.
	 * 
	 * @param powL
	 *            the left side power.
	 * @param powR
	 *            the right side power.
	 */
	public void tankDrive(double powL, double powR)
	{
		configureForTeleop();
		leftMotors.set(ControlMode.PercentOutput, (leftInverted ? -1 : 1) * powL);
		rightMotors.set(ControlMode.PercentOutput, (rightInverted ? -1 : 1) * powR);
	}

	public void clearEncoders()
	{
		leftMotors.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT);
		rightMotors.setSelectedSensorPosition(0, 0, Constants.CAN_TIMEOUT);
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
	 * Adds a two-speed gearshift to the robot drive.
	 * 
	 * @param gearshift
	 *            Two-speed gearshift object
	 * @param shiftUpSpeed
	 *            The minimum speed (in RPM) for which the gearshift should
	 *            shift to high gear
	 * @param shiftDownSpeed
	 *            The maximum speed (in RPM) for which the gearshift should
	 *            shift to low gear
	 */
	public void addShifter(TwoSpeedGearshift gearshift, double shiftUpSpeed, double shiftDownSpeed)
	{
		this.gearshift = gearshift;
		this.shiftUpSpeed = shiftUpSpeed;
		this.shiftDownSpeed = shiftDownSpeed;
	}

	public void shiftToHigh()
	{
		if (gearshift != null)
		{
			gearshift.shiftToHigh();
		}
		else
		{
			Log.info("SRXTankDrive", "You can't shift gears. The robot doesn't actually have a gearshift.");
		}
	}

	public void shiftToLow()
	{
		if (gearshift != null)
		{
			gearshift.shiftToLow();
		}
		else
		{
			Log.info("SRXTankDrive", "You can't shift gears. The robot doesn't actually have a gearshift.");
		}
	}

	public void shift()
	{
		if (gearshift != null)
		{
			gearshift.shiftToOtherGear();
		}
		else
		{
			Log.info("SRXTankDrive", "You can't shift gears. The robot doesn't actually have a gearshift.");
		}
	}

	public boolean isInHighGear()
	{
		if (gearshift != null)
		{
			return gearshift.isInHighGear();
		}
		else
		{
			Log.fatal("SRXTankDrive",
					"There is only one gear. The robot doesn't actually have a gearshift. The code that involves this is probably bad news.");
			return false;
		}
	}

	public void autoshift()
	{
		if (gearshift != null)
		{
			int rightSpeedNative = rightMotors.getSelectedSensorVelocity(0);
			int leftSpeedNative = rightMotors.getSelectedSensorVelocity(0);

			double rightSpeed = rightSpeedNative / AngularSpeed.NATIVE_UNITS_PER_100MS;
			double leftSpeed = leftSpeedNative / AngularSpeed.NATIVE_UNITS_PER_100MS;

			if (gearshift.isInHighGear() && (rightSpeed < 0 && leftSpeed > 0) || (rightSpeed > 0 && leftSpeed < 0))
			{
				gearshift.shiftToLow();
			}
			else if (!gearshift.isInHighGear() && (rightSpeed > shiftUpSpeed && leftSpeed > shiftUpSpeed))
			{
				gearshift.shiftToHigh();
			}
			else if (gearshift.isInHighGear() && (rightSpeed < shiftDownSpeed && leftSpeed < shiftDownSpeed))
			{
				gearshift.shiftToLow();
			}
		}
		else
		{
			Log.info("SRXTankDrive", "You can't shift gears. The robot doesn't actually have a gearshift.");
		}
	}

	/**
	 * Get the estimated angle that the robot has turned since the encoders were
	 * last reset, based on the relative distances of each side.
	 * 
	 * Range: [0, 360) 0 degrees is straight ahead.
	 * 
	 * @return
	 */
	public double getRobotAngle()
	{
		double leftDist = encDistanceToCm(leftMotors.getSelectedSensorPosition(0) * Angle.ROTATIONS);
		double rightDist = encDistanceToCm(rightMotors.getSelectedSensorPosition(0) * Angle.ROTATIONS);

		double difference = leftDist - rightDist;

		return RobotMath.normalizeAngle((difference / turningCircleCircumference) * Angle.ROTATIONS);
	}

	/**
	 * Convert cm of robot movement to encoder movement in degrees
	 * 
	 * @param cm
	 * @return
	 */
	public double cmToEncDegrees(double cm)
	{
		return (cm * 360) / (wheelCircumfrence * gearRatio);
	}

	/**
	 * Convert cm of robot movement to encoder rotations
	 * 
	 * @param cm
	 * @return
	 */
	public double encDistanceToCm(double encDistance)
	{
		return (encDistance / 360) * wheelCircumfrence * gearRatio;
	}

	/**
	 * Enum for how CmdMoveDistance determines when to end a move command.
	 * 
	 * @author Jamie
	 *
	 */
	public enum MoveEndMode
	{
		BOTH, // ends when both sides have reached their targets.
		EITHER, // Stops both sides when either side has reached its target.
				// Force stops the move command of the slower side.
	}

	/**
	 * Command to move each side of the drivetrain a specified distance.
	 * 
	 * Common logic shared by all of the autonomous movement commands
	 */
	public class CmdMoveDistance extends Command
	{
		// when the wheels' angular distance get within this threshold of the
		// correct value, that side is considered done
		final static double MOVEMENT_ERROR_THRESHOLD = 70 * Angle.DEGREES;

		protected double power;

		protected double leftDist, rightDist;

		protected int correctDistanceCount = 0;

		protected MoveEndMode endMode;

		boolean leftDone;
		boolean rightDone;

		boolean useScalars;
		
		/**
		 * @param leftDist
		 *            Degrees to spin the left wheel
		 * @param rightDist
		 *            Degrees to spin the right wheel
		 * @param power
		 *            motor power to move at, from 0 to 1
		 */
		public CmdMoveDistance(MoveEndMode endMode, double leftDist, double rightDist, double power, boolean useScalars,
				double timeout)
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

			double leftSpeed = (robotMaxSpeed * power * ((useScalars) ? leftSpeedScalar : 1.0));
			double rightSpeed = (robotMaxSpeed * power * ((useScalars) ? rightSpeedScalar : 1.0));

			if (leftDist == 0) {
				leftSpeed = 0;
			}
			else if (rightDist == 0) {
				rightSpeed = 0;
			}
			else if (leftDist > rightDist) {
				rightSpeed *= rightDist / leftDist;
			}
			else if (leftDist < rightDist) {
				leftSpeed *= leftDist / rightDist;
			}
			
			ControlMode leftMode = ControlMode.MotionMagic;
			ControlMode rightMode = ControlMode.MotionMagic;

			// ControlMode leftMode = ControlMode.Position;
			// ControlMode rightMode = ControlMode.Position;

			if (useScalars)
			{
				Log.info("SRXTankDrive", "Using scalars.");
			}

			// motion magic does not work well when the distance is 0
			if (leftDist == 0)
			{
				leftMode = ControlMode.Position;
			}
			if (rightDist == 0)
			{
				rightMode = ControlMode.Position;
			}

			leftMotors.configMotionCruiseVelocity((int) leftSpeed, Constants.CAN_TIMEOUT);
			leftMotors.configMotionAcceleration((int) (leftSpeed / 2.0), Constants.CAN_TIMEOUT);

			leftMotors.set(leftMode, leftDist / Angle.CTRE_MAGENC_NU);

			rightMotors.configMotionCruiseVelocity((int) rightSpeed, Constants.CAN_TIMEOUT);
			rightMotors.configMotionAcceleration((int) (rightSpeed / 2.0), Constants.CAN_TIMEOUT);

			rightMotors.set(rightMode, rightDist / Angle.CTRE_MAGENC_NU);

			Log.debug("CmdMoveDistance",
					"Distances -\nL: " + leftDist / Angle.CTRE_MAGENC_NU + " rot; R: "
							+ rightDist / Angle.CTRE_MAGENC_NU + " rot\nSpeeds-\nL: " + leftSpeed + " RPM, R: "
							+ rightSpeed + " RPM");

			try
			{
				Thread.sleep(100);
			}
			catch (InterruptedException e)
			{
				e.printStackTrace();
			}
		}

		// Make this return true when this Command no longer needs to run
		// execute()
		protected boolean isFinished()
		{
			double leftError = leftMotors.getSelectedSensorPosition(0) * Angle.CTRE_MAGENC_NU - leftDist;
			double rightError = rightMotors.getSelectedSensorPosition(0) * Angle.CTRE_MAGENC_NU - rightDist;

			Log.debug("CmdMoveDistance",
					"left pos: " + leftMotors.getSelectedSensorPosition(0) + " err: " + leftError
							+ " deg, right pos: " + rightMotors.getSelectedSensorPosition(0) + " err: " + rightError + " deg.");

			leftDone = leftDist == 0 || RobotMath.abs(leftError) < MOVEMENT_ERROR_THRESHOLD;
			rightDone = rightDist == 0 || RobotMath.abs(rightError) < MOVEMENT_ERROR_THRESHOLD;

			if (isTimedOut())
			{
				Log.unusual("CmdMoveDistance", "Autonomous Move Overtime");
				return true;
			}

			boolean isInZone;

			switch (endMode)
			{
			case BOTH:
				isInZone = leftDone && rightDone;
				break;
			case EITHER:
			default:
				isInZone = leftDone || rightDone;
				break;
			}

			if (isInZone)
			{
				++correctDistanceCount;
			}
			else
			{
				correctDistanceCount = 0;
			}

			return correctDistanceCount > 25;
		}

		// Called once after isFinished returns true
		protected void end()
		{
			Log.info("CmdMoveDistance", "Ending");

			// Now encapsulated ininitialize() because of CTRE Pheonix (v5)

			// if(leftDist == 0)
			// {
			// leftMotors.changeControlMode(TalonControlMode.MotionMagic);
			// }
			// if(rightDist == 0)
			// {
			// rightMotors.changeControlMode(TalonControlMode.MotionMagic);
			// }

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
			// do nothing
		}
	}

	/**
	 * Command to to an arc turn in the specified amount of degrees.
	 * 
	 * Runs the opposite motors from the direction provided, so turning LEFT
	 * would set the RIGHT motors.
	 * 
	 * NOTE: currently requires that the front or back wheels be omni wheels for
	 * accurate turning.
	 */
	public class CmdArcTurn extends CmdMoveDistance
	{

		/**
		 * @param degs
		 *            how far to turn in degrees. Accepts negative values.
		 * @param msec
		 *            How long the move should take. If set to 0, do not time
		 *            the move.
		 */
		public CmdArcTurn(float degs, int msec, Direction dir)
		{
			this(degs, msec, dir, .5);
		}

		// it seems like the math is consistently off by about 6%
		final static double FUDGE_FACTOR = 1.06;

		/**
		 * @param degs
		 *            how far to turn in degrees. Accepts negative values.
		 * @param msec
		 *            How long the move should take. If set to 0, do not time
		 *            the move.
		 */
		public CmdArcTurn(float degs, int msec, Direction dir, double power)
		{
			super(MoveEndMode.BOTH, 0, 0, power, false, msec);

			// this formula is explained on the info repository wiki
			double wheelAngularDist = cmToEncDegrees((2 * Math.PI * track) * (degs / 360)) * FUDGE_FACTOR;

			if (dir == Direction.RIGHT)
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
	 * Command to turn in an arc with a certain raidus for the specified amount
	 * of degrees.
	 * 
	 * Runs the opposite motors from the direction provided, so turning LEFT
	 * would set the RIGHT motors.
	 * 
	 * NOTE: currently requires that the front or back wheels be omni wheels for
	 * accurate turning.
	 */
	public class CmdFancyArcTurn extends CmdMoveDistance
	{

		/**
		 * @param degs
		 *            how far to turn in degrees. Accepts negative values.
		 * @param msec
		 *            How long the move should take. If set to 0, do not time
		 *            the move.
		 */
		public CmdFancyArcTurn(double radius, float degs, int msec, Direction dir)
		{
			this(radius, degs, msec, dir, .5);
		}

		// it seems like the math is consistently off by about 6%
		final static double FUDGE_FACTOR = 1.06;

		/**
		 * @param degs
		 *            how far to turn in degrees. Accepts negative values.
		 * @param msec
		 *            How long the move should take. If set to 0, do not time
		 *            the move.
		 */
		public CmdFancyArcTurn(double radius, float degs, int msec, Direction dir, double power)
		{
			super(MoveEndMode.BOTH, 0, 0, power, false, msec);

			// this formula is explained on the info repository wiki
			double innerAngularDist = cmToEncDegrees((degs * Math.PI / 180.0) * (radius - 0.5 * track)) * FUDGE_FACTOR;
			double outerAngularDist = cmToEncDegrees((degs * Math.PI / 180.0) * (radius + 0.5 * track)) * FUDGE_FACTOR;

			if (dir == Direction.RIGHT)
			{
				rightDist = innerAngularDist;
				leftDist = outerAngularDist;
			}
			else
			{
				rightDist = outerAngularDist;
				leftDist = innerAngularDist;
			}
		}
	}

	/**
	 * Command to to an arc turn in the specified amount of degrees.
	 * 
	 * Sets the opposite motors from the direction provided, so turning LEFT
	 * would set the RIGHT motors.
	 */
	public class CmdInPlaceTurn extends CmdMoveDistance
	{
		/**
		 * @param degs
		 *            how far to turn in degrees. Accepts negative values.
		 * @param msec
		 *            How long the move should take. If set to 0, do not time
		 *            the move
		 */
		public CmdInPlaceTurn(float degs, int msec, Direction dir)
		{
			this(degs, .5, msec, dir);
		}

		/**
		 * @param degs
		 *            how far to turn in degrees. Accepts negative values.
		 * @param msec
		 *            How long the move should take. If set to 0, do not time
		 *            the move
		 */
		public CmdInPlaceTurn(float degs, double motorPower, int msec, Direction dir)
		{
			// the encoder counts are an in-depth calculation, so we don't set
			// them until after the super constructor
			super(MoveEndMode.BOTH, 0, 0, motorPower, false, msec);

			// this formula is explained in the info repository wiki
			double wheelAngularDist = cmToEncDegrees(turningCircleCircumference * (degs / 360.0));

			if (dir == Direction.RIGHT)
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
	 * Command to move forward the given amount of centimeters. Drives straight,
	 * if you've set up your speed multipliers properly.
	 */
	public class CmdMoveForward extends CmdMoveDistance
	{

		/**
		 * @param d
		 *            how far to move. Accepts negative values.
		 * @param msec
		 *            How long the move should take. If set to 0, do not time
		 *            the move
		 */
		public CmdMoveForward(double d, int msec, boolean fullSpeed)
		{
			super(MoveEndMode.BOTH, cmToEncDegrees(d), cmToEncDegrees(d), fullSpeed ? 1 : .50, true, msec);
		}

		/**
		 * @param d
		 *            how far to move. Accepts negative values.
		 * @param msec
		 *            How long the move should take. If set to 0, do not time
		 *            the move
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
		}

	}
}