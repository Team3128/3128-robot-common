package org.team3128.testmainclasses;

import org.team3128.common.hardware.ultrasonic.IUltrasonic;
import org.team3128.common.hardware.ultrasonic.MaxSonar;
import org.team3128.common.hardware.ultrasonic.MaxSonar.Resolution;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.controller.ControllerXbox;
import org.team3128.common.multibot.MainClass;
import org.team3128.common.multibot.RobotTemplate;
import org.team3128.common.util.GenericSendableChooser;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class MainUltrasonicTest extends MainClass {

	public ListenerManager lm;

	public IUltrasonic testUltrasonic;
	
	public MainUltrasonicTest() 
	{
		lm = new ListenerManager(new Joystick(4));

		testUltrasonic = new MaxSonar(2, Resolution.MM, Port.kOnboard);
	}

	@Override
	protected void initializeRobot(RobotTemplate robotTemplate) 
	{
		robotTemplate.addListenerManager(lm);
	}

	@Override
	protected void addAutoPrograms(GenericSendableChooser<CommandGroup> autoChooser) {

	}

	@Override
	protected void initializeDisabled() {

	}

	@Override
	protected void updateDashboard() {

	}

	@Override
	protected void initializeAuto() {

	}

	@Override
	protected void initializeTeleop()
	{
		lm.addListener(ControllerXbox.XDOWN, () -> System.out.println(testUltrasonic.getDistance()));
		lm.addListener(ControllerXbox.TRIGGERL, () -> testUltrasonic.setAutoPing(false));
		lm.addListener(ControllerXbox.TRIGGERR, () -> testUltrasonic.setAutoPing(true));
	}

}
