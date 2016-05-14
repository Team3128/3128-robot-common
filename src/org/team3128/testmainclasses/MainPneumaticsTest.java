package org.team3128.testmainclasses;

import org.team3128.common.hardware.misc.Piston;
import org.team3128.common.listener.ListenerManager;
import org.team3128.common.listener.controllers.ControllerXbox;
import org.team3128.common.multibot.MainClass;
import org.team3128.common.multibot.RobotTemplate;
import org.team3128.common.util.GenericSendableChooser;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class MainPneumaticsTest extends MainClass
{
	
	public ListenerManager lmExtreme;
	
	public Piston testPiston;
	
	public Compressor compressor;
	
	public MainPneumaticsTest()
	{	
		lmExtreme = new ListenerManager(new Joystick(0));	
		
		testPiston = new Piston(new Solenoid(0), new Solenoid(1));
		testPiston.invertPiston();
		
		compressor = new Compressor();
		compressor.setClosedLoopControl(true);
	}

	protected void initializeRobot(RobotTemplate robotTemplate)
	{	
		robotTemplate.addListenerManager(lmExtreme);
		testPiston.setPistonOff();
		
		lmExtreme.nameControl(ControllerXbox.A, "PistonExtend");
		lmExtreme.nameControl(ControllerXbox.B, "PistonRetract");

		lmExtreme.nameControl(ControllerXbox.LB, "CompressorOn");
		lmExtreme.nameControl(ControllerXbox.RB, "CompressorOff");

		
		testPiston.unlockPiston();
	
		
		lmExtreme.addButtonDownListener("PistonExtend", () -> testPiston.setPistonOn());

		lmExtreme.addButtonDownListener("CompressorOn", () -> compressor.start());
		lmExtreme.addButtonDownListener("CompressorOff", () -> compressor.stop());
	}

	protected void initializeDisabled()
	{
	}

	protected void initializeAuto()
	{
	}
	
	protected void initializeTeleop()
	{	
		


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
