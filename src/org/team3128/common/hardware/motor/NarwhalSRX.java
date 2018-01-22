package org.team3128.common.hardware.motor;

/**
 * 
 * An extension of the Cross The Road Phoenix Library's {@link NarwhalSRX} class
 * in order to make accounting for flipped motors, encoders, or both slightly
 * easier.
 * 
 * @author Ronak
 *
 */
public class NarwhalSRX extends com.ctre.phoenix.motorcontrol.can.TalonSRX {
	public enum Reverse {
		NONE(false, false, "None"),
		ENCODER(false, true, "Encoder"),
		OUTPUT(true, false, "Output"),
		BOTH(true, true, "Both");
		
		private boolean setInverted, setSensorPhase;
		private String name;
		
		Reverse(boolean setInverted, boolean setSensorPhase, String name) {
			this.setInverted = setInverted;
			this.setSensorPhase = setSensorPhase;
			this.name = name;
		}
		
		public String getName() {
			return name;
		}
	}
	
	public Reverse currentMode;
	
	public Reverse autonomousMode, teleopMode;
	
	/**
	 * Creates a Talon SRX with the CAN ID, also the reverse modes for autonomous and teleop.
	 * 
	 * @param deviceNumber The CAN ID of the Talon
	 * @param autonomous The {@link Reverse} mode in autonomous
	 * @param teleop The {@link Reverse} mode in teleop
	 */
	public NarwhalSRX(int deviceNumber, Reverse autonomous, Reverse teleop) {
		super(deviceNumber);
		
		currentMode = Reverse.NONE;
		
		setAutnomousReverseMode(autonomous);
		setReverseMode(teleop);
	}
	
	public NarwhalSRX(int deviceNumber) {
		super(deviceNumber);
		
		currentMode = Reverse.NONE;
	}
	
	/**
	 * Sets the {@link Reverse} mode.
	 * 
	 * @param mode
	 */
	public void setReverseMode(Reverse mode) {
		currentMode = mode;
		
		setInverted(mode.setInverted);
		setSensorPhase(mode.setSensorPhase);
	}
	
	public void setAutnomousReverseMode(Reverse mode) {
		autonomousMode = mode;
	}
	
	public void setTeleopReverseMode(Reverse mode) {
		teleopMode = mode;
	}
	
	public void configureAutonomous() {
		setReverseMode(autonomousMode);
	}
	
	public void configureTeleop() {
		setReverseMode(teleopMode);
	}
}
