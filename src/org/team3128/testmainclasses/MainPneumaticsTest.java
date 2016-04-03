package org.team3128.testmainclasses;

import org.team3128.common.hardware.misc.Piston;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.controller.ControllerXbox;
import org.team3128.common.multibot.MainClass;
import org.team3128.common.multibot.RobotTemplate;
import org.team3128.common.util.GenericSendableChooser;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class MainPneumaticsTest extends MainClass
{
	
	public ListenerManager listenerManagerExtreme;
	
	public Piston testPiston;
	
	public Compressor compressor;
	
	public MainPneumaticsTest()
	{	
		listenerManagerExtreme = new ListenerManager(new Joystick(0));	
		
		testPiston = new Piston(new Solenoid(0), new Solenoid(1));
		testPiston.invertPiston();
		
		compressor = new Compressor();
		compressor.setClosedLoopControl(true);
	}

	protected void initializeRobot(RobotTemplate robotTemplate)
	{	
		robotTemplate.addListenerManager(listenerManagerExtreme);
		testPiston.setPistonOff();
	}

	protected void initializeDisabled()
	{
	}

	protected void initializeAuto()
	{
	}
	
	protected void initializeTeleop()
	{	
		testPiston.unlockPiston();
		
		listenerManagerExtreme.addListener(ControllerXbox.ADOWN, () -> testPiston.setPistonOn());
		
		listenerManagerExtreme.addListener(ControllerXbox.BDOWN, () -> testPiston.setPistonOff());
		
		listenerManagerExtreme.addListener(ControllerXbox.LBDOWN, () -> compressor.stop());
		listenerManagerExtreme.addListener(ControllerXbox.RBDOWN, () -> compressor.start());

	}

	@Override
	protected void addAutoPrograms(GenericSendableChooser<CommandGroup> autoChooser)
	{
	}

	@Override
	protected void updateDashboard()
	{
	}
}
