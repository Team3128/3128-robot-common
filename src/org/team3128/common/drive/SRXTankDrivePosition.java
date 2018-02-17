package org.team3128.common.drive;

import org.team3128.common.util.RobotMath;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

/**
 * Experimental class in order to constantly integrate encoder and gyroscope
 * readings in order to determine the displacement of the robot from its initial
 * position.
 * 
 * @author Ronak
 *
 */
public class SRXTankDrivePosition {
	private ADXRS450_Gyro gyro;
	private TalonSRX leftDriveMotors, rightDriveMotors;

	/**
	 * The circumference of the wheels, in centimeters
	 */
	private double wheelCirc;

	private Thread updateThread;

	private int dt = 100;

	/**
	 * The x and y displacement of the robot with respect to the last reset, in
	 * centimeters.
	 */
	private double xPosition, yPosition;

	/**
	 * The gyro return angle that we should assume to be zero, since gyroscopes
	 * can't reset with the robot on.
	 */
	private double zeroAngle;

	public SRXTankDrivePosition(TalonSRX leftDriveMotors, TalonSRX rightDriveMotors, ADXRS450_Gyro gyro,
			double wheelCirc) {
		this.leftDriveMotors = leftDriveMotors;
		this.rightDriveMotors = rightDriveMotors;

		this.wheelCirc = wheelCirc;

		this.gyro = gyro;

		setPosition(0, 0, 0);

		updateThread = new Thread(() -> {
			while (true) {
				double leftVelocity = this.wheelCirc * this.leftDriveMotors.getSelectedSensorVelocity(0) * 10 / 4096.0;
				double rightVelocity = this.wheelCirc * this.rightDriveMotors.getSelectedSensorVelocity(0) * 10 / 4096.0;

				double gyroAngle = gyro.getAngle();

				// the speed (scalar) of the center of rotation of the robot
				double crSpeed = Math.abs(
						rightVelocity * (leftVelocity - rightVelocity) * (0.5 + 1.0 / (leftVelocity - rightVelocity)));

				double dx = crSpeed * RobotMath.cos(90 - gyroAngle) * dt / 1000.0;
				double dy = crSpeed * RobotMath.cos(gyroAngle) * dt / 1000.0;

				incrementPosition(dx, dy);

				try {
					Thread.sleep(dt);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		});

		updateThread.start();
	}

	public void setPosition(double x, double y, double theta) {
		xPosition = x;
		yPosition = y;

		zeroAngle = gyro.getAngle() - theta;
	}

	private synchronized void incrementPosition(double dx, double dy) {
		xPosition += dx;
		yPosition += dy;
	}

	public double getXPosition() {
		return xPosition;
	}

	public double getYPosition() {
		return yPosition;
	}

	public double getAngle() {
		return gyro.getAngle() - zeroAngle;
	}

}
