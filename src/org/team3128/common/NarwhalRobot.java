package org.team3128.common;

import java.util.ArrayList;

import org.team3128.common.listener.ListenerManager;
import org.team3128.common.util.GenericSendableChooser;
import org.team3128.common.util.Log;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.communication.FRCNetworkCommunicationsLibrary;
import edu.wpi.first.wpilibj.communication.FRCNetworkCommunicationsLibrary.tInstances;
import edu.wpi.first.wpilibj.communication.FRCNetworkCommunicationsLibrary.tResourceType;
import edu.wpi.first.wpilibj.communication.UsageReporting;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * THIS FILE SHOULD NOT BE MODIFIED
 * --------------------------------
 * It serves as a link to the main class.
 * Events triggered here will be forwarded there.
 *
 * Do not call these functions under any circumstances. Do not modify this
 * class under any circumstances.
 *
 * AUTOGENERATED. DO NOT EDIT UNDER PENALTY OF 42.
 *
 * THIS FILE IS YOUR SOUL.
 */

public abstract class NarwhalRobot extends RobotBase 
{
	//functions the the robot class should implement
	//---------------------------------------------------------------------------------
	/**
	 * Use this to construct all of the hardware objects, like motors and pistons
	 */
	protected abstract void constructHardware();
	
	/**
	 * Use this to add all of the teleop listeners.
	 * 
	 * Make sure to add all of your ListenerManagers to this class to be ticked using {@link addListenerManager()}
	 */
	protected abstract void setupListeners();
	
	/**
	 * Construct your autonomous programs and add them to this SendableChooser here.
	 * 
	 * This function will called multiple times.
	 */
	protected void constructAutoPrograms(GenericSendableChooser<CommandGroup> programChooser) {}
	
	/**
	 * Called every time teleop is started from the driver station
	 */
	protected abstract void teleopInit();
	
	/**
	 * Called in teleop mode every time new control data arrives from the DS.
	 * 
	 * Called AFTER listeners are invoked.
	 */
	protected void teleopPeriodic() {}
	
	/**
	 * Called every time autonomous starts.
	 * 
	 * Called BEFORE the autonomous sequence is started.
	 */
	protected abstract void autonomousInit();
	
	/**
	 * Called every loop during autonomous. You probably won't need to use this, but it's there anyway.
	 */
	protected void autonomousPeriodic() {}
	
	/**
	 * Called when the robot enters the disabled state.
	 */
	protected void disabledInit() {}
	
	/**
	 * Called every cycle while the robot is disabled.
	 */
	protected void disabledPeriodic() {}
	
	/**
	 * Called when the robot enters test mode.
	 */
	protected void testInit() {}
	
	/**
	 * Called every cycle while the robot is in test mode.
	 */
	protected void testPeriodic() {}
	
	/**
	 * Use this function to read and write data from the SmartDashboard.
	 * It is called asynchronously, no matter what mode the robot is in.
	 */
	protected void updateDashboard() {}
	
	//---------------------------------------------------------------------------------

	ArrayList<ListenerManager> listenerManagers = new ArrayList<ListenerManager>();
	GenericSendableChooser<CommandGroup> autoChooser;
	
	final static int dashboardUpdateWavelength = 100; //NetworkTables transmits every 100ms... I think
	
	Thread dashboardUpdateThread;
	
	boolean wasInAutonomous = false;
	
	private boolean disabledInitialized = false;
	private boolean autonomousInitialized = false;
	private boolean teleopInitialized = false;
	private boolean testInitialized = false;
	
	public void startCompetition()
	{
	    UsageReporting.report(tResourceType.kResourceType_Framework, tInstances.kFramework_Iterative);
        LiveWindow.setEnabled(false);

        // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        Log.info("NarwhalRobot", "Welcome to the FRC Team 3128 Common Library version 3.2!");
        Log.info("NarwhalRobot", "Initializing Base Robot...");
	    
        try
        {
        	constructHardware();
        }
        catch(RuntimeException ex)
        {
        	Log.fatalException("NarwhalRobot", "Exception constructing hardware", ex);
        	fail();
        }
        
        try
        {
        	setupListeners();
        }
        catch(RuntimeException ex)
        {
        	Log.fatalException("NarwhalRobot", "Exception seting up listeners", ex);
        	fail();
        }
        
        //construct auto programs the first time
		setupAutoChooser();

        
        Log.info("NarwhalRobot", "Starting Dashboard Update Thread...");
        dashboardUpdateThread = new Thread(this::updateDashboardLoop, "Dashboard Update Thread");
        dashboardUpdateThread.start();
        
        Log.info("NarwhalRobot", "Initialization Done!");

        // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        // Tell the DS that the robot is ready to be enabled
        FRCNetworkCommunicationsLibrary.FRCNetworkCommunicationObserveUserProgramStarting();
        // loop forever, calling the appropriate mode-dependent function
        
		while (true) {
			// Call the appropriate function depending upon the current robot mode
			if (isDisabled()) {
				// call DisabledInit() if we are now just entering disabled mode from
				// either a different mode or from power-on
				if (!disabledInitialized) {
					LiveWindow.setEnabled(false);
					
					if(autonomousInitialized)
					{
						Log.info("NarwhalRobot", "Re-constructing autonomous sequences");
						//re-construct the autonomous programs
						setupAutoChooser();
					}
					
					disabledInit();
					disabledInitialized = true;
					// reset the initialization flags for the other modes
					autonomousInitialized = false;
					teleopInitialized = false;
					testInitialized = false;
				}
				if (nextPeriodReady()) {
					FRCNetworkCommunicationsLibrary.FRCNetworkCommunicationObserveUserProgramDisabled();
					disabledPeriodic();
				}
			} else if (isTest()) {
				// call TestInit() if we are now just entering test mode from either
				// a different mode or from power-on
				if (!testInitialized) {
					LiveWindow.setEnabled(true);
					testInit();
					testInitialized = true;
					autonomousInitialized = false;
					teleopInitialized = false;
					disabledInitialized = false;
				}
				if (nextPeriodReady()) {
					FRCNetworkCommunicationsLibrary.FRCNetworkCommunicationObserveUserProgramTest();
					testPeriodic();
				}
			} else if (isAutonomous()) {
				// call autonomousInit() if this is the first time
				// we've entered autonomous_mode
				if (!autonomousInitialized) {
					LiveWindow.setEnabled(false);
					// KBS NOTE: old code reset all PWMs and relays to "safe values"
					// whenever entering autonomous mode, before calling
					// "Autonomous_Init()"
					
			        Log.info("NarwhalRobot", "Initializing Autonomous...");
			        
					autonomousInit(); //we probably want to call this first, as it may set preconditions for autonomous
					runAutoProgram(); 
					autonomousInitialized = true;
					testInitialized = false;
					teleopInitialized = false;
					disabledInitialized = false;
					
			        Log.info("NarwhalRobot", "Auto Initialization Done!");
				}
				//I claim we don't need to wait for new control data, we can just go
				FRCNetworkCommunicationsLibrary.FRCNetworkCommunicationObserveUserProgramAutonomous();
				Scheduler.getInstance().run();
				autonomousPeriodic();
				
			} else {
				// call Teleop_Init() if this is the first time
				// we've entered teleop_mode
				if (!teleopInitialized) 
				{
					LiveWindow.setEnabled(false);
					
			        Log.info("NarwhalRobot", "Initializing Teleop...");
					
					teleopInit();
					recountAllControls();
					teleopInitialized = true;
					testInitialized = false;
					autonomousInitialized = false;
					disabledInitialized = false;
					
			        Log.info("NarwhalRobot", "Teleop Initialization Done!");
				}
				if (nextPeriodReady()) 
				{
					FRCNetworkCommunicationsLibrary.FRCNetworkCommunicationObserveUserProgramTeleop();
					tickListenerManagers();
					teleopPeriodic();
				}
			}
			
			if(!autonomousInitialized) //don't need to wait for control data if we are in autonomous
			{
				m_ds.waitForData();
			}
		}
	}
	
	/**
	 * Determine if the appropriate next periodic function should be called. Call
	 * the periodic functions whenever a packet is received from the Driver
	 * Station, or about every 20ms.
	 */
	private boolean nextPeriodReady() {
		return m_ds.isNewControlData();
	}
	
	/**
	 * Call this when the robot code has encountered a severe error and must stop.  
	 * Log the reason, if you can.
	 * Hopefully avoids the issue where longs don't get transmitted when the program crashes.
	 */
	protected void fail()
	{
		Log.fatal("NarwhalRobot", "Critical error.  Robot exiting.");

		//give the failure message time to get to the driver station
		try
		{
			Thread.sleep(200);
		}
		catch (InterruptedException e)
		{
			e.printStackTrace();
		}
		
		System.exit(7);
	}
	
	private void setupAutoChooser()
	{
        Log.info("NarwhalRobot", "Setting Up Autonomous Chooser...");
		autoChooser = new GenericSendableChooser<>();
        constructAutoPrograms(autoChooser);
        
        if(autoChooser.getLength() > 1)
        {
            SmartDashboard.putData("autoChooser", autoChooser);
        }
	}
	
	private void runAutoProgram()
	{
		
		//we need to be error tolerant here, this has bit me in the past
		CommandGroup autoProgram = null;
		
		if(autoChooser.getLength() > 0)
		{
			autoProgram = autoChooser.getSelected();
		}
		
		if(autoProgram == null)
		{
			Log.recoverable("NarwhalRobot", "Can't start autonomous, there is no sequence to run.  "
					+ "You either didn't provide any sequences, or didn't set a default and didn't select one on the dashboard.");
		}
		else
		{
			autoProgram.start();
		}
	}
	
    // ARE YOU CHANGING THINGS?
    
    /**
     * This function is run in its own thread to call main.updateDashboard()
     */
    private void updateDashboardLoop()
    {
		Log.info("NarwhalRobot", "Dashboard Update Thread starting");
    	while(true)
    	{
    		updateDashboard();
    		
    		try
			{
				Thread.sleep(dashboardUpdateWavelength);
			} 
    		catch (InterruptedException e)
			{
    			Log.info("NarwhalRobot", "Dashboard Update Thread shutting down");
				return;
			}
    	}
    	
    }
    
    // TURN BACK NOW.
    // YOUR CHANGES ARE NOT WANTED HERE.
    
    /**
     * Add a listener manager to the list of ones to be ticked in teleopPeriodic().
     * @param manager
     */
    public void addListenerManager(ListenerManager manager)
    {
    	listenerManagers.add(manager);
    }
    
    // YOU'D BETTER NOT CHANGE ANYTHING
   
    //works around an annoying (though understandable) WPILib issue:
    //if the DS is not connected, it has no idea how many buttons/axes/POVs there are on a joystick
    //so it seems to just make up a number
    //so when we start teleop we recount them, since the DS must be connected by now.
    private void recountAllControls()
    {
    	for(ListenerManager manager : listenerManagers)
    	{
    		manager.recountControls();
    	}
    }

    // DO YOU REALLY WANT TO MODIFY YOUR SOUL?
    public void tickListenerManagers()
    {        
    	long startTime = System.currentTimeMillis();
    	
    	for(ListenerManager manager : listenerManagers)
    	{
    		manager.tick();
    	}
    	
    	Log.debug("NarwhalRobot", "ListenerManager ticking took " + (System.currentTimeMillis() - startTime) + " ms.");
    }
    
}

