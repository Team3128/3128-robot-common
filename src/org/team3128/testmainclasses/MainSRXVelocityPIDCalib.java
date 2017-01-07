package org.team3128.testmainclasses;

import org.team3128.common.NarwhalRobot;
import org.team3128.common.util.Log;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MainSRXVelocityPIDCalib extends NarwhalRobot 
{
	
	
	//change these to match your robot
	static int CAN_ID = 5;
	static int ENCODER_CPR_NATIVE_UNITS = 1024 * 4;
	static double TESTING_MOTOR_POWER = .8;
	
	private enum State
	{
		TEST_DIR,
		CALIB_FEEDFORWARD,
		CALIB_PID
	}
	
	final static String TAG = "SRX Velocity Calibrator";
	
	CANTalon testSRX;
	
	private State state = State.TEST_DIR;
	
	private Joystick joy;
	
	// estimated max speed of the motor
	private double motorMaxRPM;
	
	@Override
	protected void constructHardware() 
	{
		testSRX = new CANTalon(CAN_ID);
		testSRX.changeControlMode(TalonControlMode.PercentVbus);
		testSRX.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		testSRX.setPID(0, 0, 0, 0, 0, 0, 0);
		testSRX.setVoltageRampRate(12); // V per sec
		testSRX.reverseSensor(true);
		
		joy = new Joystick(0);
		
		Log.info(TAG, "Started");
		Log.info(TAG, "Testing that encoder is not reversed");
	}

	@Override
	protected void setupListeners() {
		// TODO Auto-generated method stub

	}

	@Override
	protected void teleopInit() {
		// TODO Auto-generated method stub

	}

	@Override
	protected void autonomousInit() {
		// TODO Auto-generated method stub

	}
	
	@Override
	protected void teleopPeriodic()
	{
		switch(state)
		{
		case TEST_DIR:
			testSRX.set(TESTING_MOTOR_POWER);
			Log.info(TAG, "speed: " + testSRX.getSpeed());
			if(testSRX.getSpeed() > 0)
			{
				state = State.CALIB_FEEDFORWARD;
				Log.info(TAG, "Encoder setup correct.  Calibrating Feedforward...");
				
			}
			else if(testSRX.getSpeed() < 0)
			{
				Log.recoverable(TAG, "The encoder is backwards!  Please reverse it and try again");
				testSRX.disable();
			}
			break;
		case CALIB_FEEDFORWARD:
			double velocityNativeUnits = testSRX.getSpeed() * (ENCODER_CPR_NATIVE_UNITS/60);
			double feedforward = (1023 * TESTING_MOTOR_POWER) / velocityNativeUnits; 
			Log.info(TAG, "Feedforward: " + feedforward + " (Press Trigger to use this value and continue)");
			
			if(joy.getTrigger())
			{
				motorMaxRPM = testSRX.getSpeed();
				SmartDashboard.putNumber("P", 0);
				SmartDashboard.putNumber("I", 0);
				SmartDashboard.putNumber("D", 0);
				
				testSRX.setF(feedforward);
				state = State.CALIB_PID;
				
				testSRX.changeControlMode(TalonControlMode.Speed);
				
				Log.info(TAG, "Please calibrate PID constants. The joystick controls speed.");
				Log.info(TAG, "When these PID values work, you're done!");

			}
			break;
		case CALIB_PID:
			double targetRPM = motorMaxRPM * joy.getY();
			
			SmartDashboard.putNumber("Target RPM: ", targetRPM);
			SmartDashboard.putNumber("Error (RPM)", testSRX.getClosedLoopError());
			
			testSRX.set(targetRPM);
			
			testSRX.setP(SmartDashboard.getNumber("P"));
			testSRX.setI(SmartDashboard.getNumber("I"));
			testSRX.setD(SmartDashboard.getNumber("D"));
		
		}
	}
	
	@Override
	protected void updateDashboard()
	{
		SmartDashboard.putNumber("Speed (RPM)", testSRX.getSpeed());
	}

}
