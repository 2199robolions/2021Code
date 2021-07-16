package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Conveyer.ConveyerState;
import edu.wpi.first.wpilibj.shuffleboard.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

	// ERROR CODES
	public static final int FAIL = -1;
	public static final int PASS = 1;
	public static final int DONE = 2;
	public static final int CONT = 3;

	// Constants
	private final int LED_DELAY = 15;

	// Variables
	private int     climberStatus;
	private int     colorWheelStatus;
	private int     backupVerticalCount;
	private int     colorMode       = 0;
	private int     ledCurrent      = 0;
	private int     colorWheelSpin  = 0;
	private int     delaySeconds    = 0;
	private int     autoStatus      = Robot.CONT; 
	private long    startTime       = 0;
	private long    endTime         = 0;
	private boolean climbEnabled    = false;
	private boolean climberRetract  = false;
	private boolean enableTopPiston = false;
	private Climber.ClimberState climberState = Climber.ClimberState.ALL_ARMS_DOWN;

	// OBJECTS
	private LedLights  ledLights;
	private Controller controller;
	private Wheels     wheels;
	private Shooter    shooter;
	private ColorWheel colorWheel;
	private Conveyer   conveyer;
	private Grabber    grabber;
	private Climber    climber;
	private Auto       auto;

	//Setting Up WheelMode
	private Wheels.WheelMode wheelMode;
	private int targetingStatus;

	/**
	 * Auto Choices
	 */
	//Position
	private static final String kDefaultAuto      = "Default";
	private static final String kCustomAutoRight  = "Right";
	private static final String kCustomAutoCenter = "Center";
	private static final String kCustomAutoLeft   = "Left";
	private static final String kCustomAutoLRC    = "L/R/C Simple";

	private String m_positionSelected;
	private final SendableChooser<String> m_chooser = new SendableChooser<>();

	//Delay
	private static final String kDefaultTime    = "0";
	private static final String kCustomChooser2 = "2";
	private static final String kCustomChooser4 = "4";
	private static final String kCustomChooser6 = "6";

	private String m_delaySelected;
	private final SendableChooser<String> m_delayChooser = new SendableChooser<>();

	// Alliance Color
	private static final String kDefaultColor   = "Default";
	private static final String kBlue           = "Blue";
	private static final String kRed            = "Red";

	private String alliance;
	private final SendableChooser<String> allianceColor = new SendableChooser<>();


	//Enumeration of galactic search paths
	
	/** 
	 * Constructor
	 */
	public Robot() {
		ledLights  = new LedLights();
		controller = new Controller();
		wheels     = new Wheels(ledLights);
		shooter    = new Shooter();
		//lidar      = new Lidar(0);
		colorWheel = new ColorWheel();
		conveyer   = new Conveyer();
		grabber    = new Grabber();
		climber    = new Climber();
		auto       = new Auto(ledLights, wheels, shooter, conveyer, grabber, colorWheel);

		// Set Variables
		colorMode         = 0;
		ledCurrent        = 0;

		// Set Different Staus Cues
		colorWheelStatus  = Robot.DONE;
		climberStatus     = Robot.DONE;
		wheelMode         = Wheels.WheelMode.MANUAL;
	}

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		//Auto Positions
		m_chooser.addOption(kDefaultAuto , kDefaultAuto);
		m_chooser.addOption(kCustomAutoRight, kCustomAutoRight);
		m_chooser.addOption(kCustomAutoCenter, kCustomAutoCenter);
		m_chooser.addOption(kCustomAutoLeft, kCustomAutoLeft);
		m_chooser.addOption(kCustomAutoLRC, kCustomAutoLRC);

		//Default Auto Position
		m_chooser.setDefaultOption(kDefaultAuto, kDefaultAuto);
		SmartDashboard.putData("Auto Positions", m_chooser);

		//Auto Delay
		m_delayChooser.addOption(kDefaultTime,    "0");
		m_delayChooser.addOption(kCustomChooser2, "2");
		m_delayChooser.addOption(kCustomChooser4, "4");
		m_delayChooser.addOption(kCustomChooser6, "6");
		
		//Default Auto Delay
		m_delayChooser.setDefaultOption(kDefaultTime, kDefaultTime);
		SmartDashboard.putData("Auto delay", m_delayChooser);

		//Alliance Color Options
		allianceColor.addOption(kDefaultColor, kDefaultColor);
		allianceColor.addOption(kBlue, kBlue);
		allianceColor.addOption(kRed, kRed);

		//Alliance Select
		allianceColor.setDefaultOption(kDefaultColor, kDefaultColor);
		SmartDashboard.putData("Alliance Color", allianceColor);

		//Testing shooter powers
		SmartDashboard.putNumber("Input Power", 0.75);
		
		alliance = allianceColor.getSelected();	

		//Set Limelight to On
		wheels.changeLimelightLED(Wheels.LIMELIGHT_ON);
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use
	 * this for items like diagnostics that you want ran during disabled,
	 * autonomous, teleoperated and test.
	 *
	 * <p>This runs after the mode specific periodic functions, but before
	 * LiveWindow and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		//
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		m_positionSelected  = m_chooser.getSelected();
		m_delaySelected     = m_delayChooser.getSelected();
		delaySeconds        = Integer.parseInt(m_delaySelected);
		autoStatus          = Robot.CONT;  
		startTime           = System.nanoTime();  
		double yaw			= wheels.getYaw();

		System.out.println("Delay: "    + delaySeconds);
		System.out.println("Position: " + m_positionSelected);

		/*
		if((yaw > 33) && (yaw < 42)){
			galacticPath = GalacticPath.A_RED;
			SmartDashboard.putString("Path", "A_RED");
		} else if ((yaw > 13) && (yaw < 27)){
			galacticPath = GalacticPath.B_RED;
			SmartDashboard.putString("Path", "B_RED");			
		} else if ((yaw > -7) && (yaw < 7)){
			galacticPath = GalacticPath.A_BLUE;
			SmartDashboard.putString("Path", "A_BLUE");			
		} else if ((yaw < -12) && (yaw > -26)){
			galacticPath = GalacticPath.B_BLUE;
			SmartDashboard.putString("Path", "B_BLUE");	
		}*/

	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		if (autoStatus == Robot.CONT) {
			if (m_positionSelected.equals(kCustomAutoLRC) == true) {
				autoStatus = auto.autoSimple(delaySeconds);
				//System.out.println("Running simple auto");
			} else if (m_positionSelected.equals(kCustomAutoRight) == true){
				autoStatus = auto.autoRightFull(delaySeconds);
				//System.out.println("Running right auto");
			} else if (m_positionSelected.equals(kCustomAutoCenter) == true){
				autoStatus = auto.autoCenterFull(delaySeconds);
			}
			SmartDashboard.putNumber("Time", ((System.nanoTime() - startTime)/1000000000.0) );
			
		} else if (autoStatus == Robot.DONE){
			endTime = System.nanoTime();
			//System.out.println("Elapsed Time: " + ((endTime - startTime)/1000000000.0));	
		}
	}


	/**
	 * This function is called periodically during autonomous.
	 */
//	@Override
	public void CompetitionautonomousPeriodic() {

		if (autoStatus == Robot.CONT) {
			switch (m_positionSelected) {
				case "Right":
					autoStatus = auto.rightAuto(delaySeconds);
					break;
				case "Left":
					autoStatus = auto.leftAuto(delaySeconds);
					break;
				case "Center":
					autoStatus = auto.centerAuto(delaySeconds);
					break;
				case kCustomAutoLRC:
					autoStatus = auto.leftRightCenterSimpleForwardAuto(delaySeconds);
					break;
				default :
					// Put default auto code here
					autoStatus = auto.centerAuto(delaySeconds);
					break;
			}
		}
		
	}

	@Override
	public void teleopInit() {
		startTime           = System.nanoTime();   
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {

		/*****************************************************************
		 *     BALL MOVEMENT
		 *****************************************************************/
		ballControl();
		
		/*****************************************************************
		 *     WHEELS CONTROL
		 *****************************************************************/
		wheelControl();

		/*****************************************************************
		 *     COLOR WHEEL
		 *****************************************************************/
		colorWheelControl();

		/*****************************************************************
		 *     Climber
		 *****************************************************************/
		climberControl();

		/*****************************************************************
		 * 	   Timer
		 */
		SmartDashboard.putNumber("Time", ((System.nanoTime() - startTime)/1000000000.0));
	}

	

	public void testInit()  {
		// Placeholder
		System.out.println("Start test mode...");
		System.out.println("record time...");
		autoStatus = Robot.CONT;
		startTime = System.nanoTime();  
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		//wheels.forward(-3.0, 0.0);
		//System.out.println(wheels.getDistance());
		//System.out.println(wheels.getCameraMountingAngle(12));
		//System.out.println("Wheels Rotate " + stat);

		//shooter.autoShooterControl(Shooter.ShootLocation.OFF);
		//auto.shooterTest();

		//auto.testAll();
		//wheels.testPidError();
		//wheels.testYaw();
		//wheels.testPid();
		//wheels.testArcadeRotation();
		//wheels.testArcadeRotation();
		//wheels.testRotation(-90);

		/*
		System.out.println("Power: " + tempPower);	
		conveyer.manualVerticalControl(controller.getVerticalBeltState());   
		conveyer.manualHorizontalControl(controller.getHorizonalBeltState());   
		*/

		if (autoStatus == CONT) {
			autoStatus = wheels.rotate(180);
			//SmartDashboard.putNumber("Time", ((System.nanoTime() - startTime)/1000000000.0) );
		} else if (autoStatus == DONE) {
			System.out.println("Done: " + (System.nanoTime() - startTime));
		}
		
		/*
		double tempPower = 0.65;
		tempPower = SmartDashboard.getNumber("Input Power", 0.5);
		shooter.testShoooter(tempPower);
		conveyer.manualVerticalControl(Conveyer.ConveyerState.FORWARD);   
		conveyer.manualHorizontalControl(Conveyer.ConveyerState.FORWARD);   
		*/


	}



	/**********************************************************
	 * 
	 * Climber Control
	 * 
	 **********************************************************/
	private void climberControl() {
		boolean allArmsUp;
		boolean topArmDown;
		double  climberPower;

		// only button as of now for climber
		allArmsUp    = controller.climberAllArmsUp();
		topArmDown   = controller.climberTopArmDown();
		climberPower = controller.getClimberPower();


		// All arms down
		if (climberState == Climber.ClimberState.ALL_ARMS_DOWN) {
			if (allArmsUp == true) {
				climberStatus = auto.climberDeploy(climber);
				climberState = Climber.ClimberState.START_ARMS_UP;
			}
		}
		// ready to deploy cliber arms
		else if (climberState == Climber.ClimberState.START_ARMS_UP) {
			climberStatus = auto.climberDeploy(climber);
			if ( climberStatus == Robot.DONE ) {
				climberState = Climber.ClimberState.ALL_ARMS_UP;
			}
		}
		// All arms are up
		else if (climberState == Climber.ClimberState.ALL_ARMS_UP) {
			if (topArmDown == true) {
				climber.topArmDown();
				climberState = Climber.ClimberState.TOP_ARM_DOWN;
			}
			// attempting to redeploy arms
			else if (allArmsUp == true) {
				climberState = Climber.ClimberState.START_ARMS_UP;
			}
		}
		// Top arm down ready to climb unless you need to redeploy climber arms
		else if (climberState == Climber.ClimberState.TOP_ARM_DOWN) {
			if (climberPower > 0) {
				climber.pullRobotUp(climberPower);
				climber.climberDown();
				climberState = Climber.ClimberState.CLIMB;
			}
			else if (allArmsUp == true) {
				climberState = Climber.ClimberState.START_ARMS_UP;
			}
		}
		// you are climbed
		else if (climberState == Climber.ClimberState.CLIMB) {
			climber.pullRobotUp(climberPower);
		}
	}



	 /**********************************************************
	 * 
	 * Wheel Control
	 * 
	 **********************************************************/
	private void wheelControl() {
		//Target Lock
		if ( controller.enableTargetLock() == true ) {
			wheelMode = Wheels.WheelMode.TARGET_LOCK;
		}

		//Target Lock Auto Kill
		if ( controller.autoKill() == true ) {
			wheelMode = Wheels.WheelMode.MANUAL;
		}

		//Manual Drive
		if ( wheelMode == Wheels.WheelMode.MANUAL ) {
			wheels.controllerDrive(controller.getForwardPower(),
								   controller.getClockwiseRotation());

			ledCurrent++;
	
			if (ledCurrent >= LED_DELAY) {
				ledCurrent = 0;

				ledLights.defaultMode(alliance);
			}
		}
		else if ( wheelMode == Wheels.WheelMode.TARGET_LOCK ) {
			ledCurrent = 0;

			//PID Targeting when in Target Lock Mode
			targetingStatus = wheels.limelightPIDTargeting( Wheels.TargetPipeline.TEN_FOOT );

			if ( targetingStatus == Robot.DONE ) {
				wheelMode = Wheels.WheelMode.MANUAL;
			}
			else if ( targetingStatus == Robot.FAIL ) {
				wheelMode = Wheels.WheelMode.MANUAL;
			}
		}
	
		//Shift Gears For Wheels
		if ( controller.shiftGears() == true ) {
			wheels.gearShift();
		}
	}


	 /**********************************************************
	 * 
	 * Color Wheel Control
	 * 
	 **********************************************************/
	private void colorWheelControl() {
		// Deploy Color Sensor
		if ( controller.colorDeployRetract() == true ) {
			colorWheel.deployRetract();
		}

		// Rotate the Wheel
		double wheelPower = controller.colorWheelManualControl();
		
		if ( controller.colorRotate() == true ) {
			colorWheelSpin = 1;
		}

		/**
		 * Auto Rotate
		 */
		if ( colorWheelSpin == 1 ) {
			if (colorMode == 0) {
				colorWheelStatus = colorWheel.colorWheelSpin();
			}

			if ( colorMode == 1 ) {
				colorWheelStatus = colorWheel.colorWheelColorMatch();
			}
		}

		/**
		 * Determine ColorWheel Status
		 */
		if ( colorMode == 0 ) {
			if (colorWheelStatus == Robot.DONE) {
				colorMode = 1;
				colorWheelSpin = 0;
			}
		}

		if ( colorMode == 1 ) {
			if (colorWheelStatus == Robot.DONE) {
				colorMode = 0;
				colorWheelSpin = 0;
			}
		}

		/**
		 * Manual Rotate
		 */
		if ( colorWheelStatus == Robot.DONE ) {
			if (wheelPower >= 0.1) {
				colorWheel.rotateColorWheel(wheelPower);
			}
			else {
				colorWheel.rotateColorWheel( 0.0 );
			}
		}
	}


	/**********************************************************
	 * 
	 * Ball Control
	 * 
	 **********************************************************/
	private void ballControl() {
		boolean grabberDeployRetract;
		boolean horizontalPistonDeployRetract;
		boolean hailMary;
		boolean shooterEnable;
		boolean trenchShot;
		ConveyerState horizontalBeltState;
		ConveyerState verticalBeltState;
		Grabber.GrabberDirection grabberDirection;
		
		
		// Get setting from Xbox & Joystick controllers
		grabberDeployRetract          = controller.grabberDeployRetract();
		grabberDirection              = controller.getGrabberDir();
		horizontalPistonDeployRetract = controller.getForwardingPressed();
		horizontalBeltState           = controller.getHorizonalBeltState();
		verticalBeltState             = controller.getVerticalBeltState();
		hailMary                      = controller.hailMary();
		shooterEnable                 = controller.enableShooter();
		trenchShot                    = controller.enableTrenchShot();
		
		
		/*****   Grabber Deploy Retract   *****/
		if (grabberDeployRetract == true) {
			grabber.deployRetract();
		}
		
		
		/******   Grabber motor Forward, Reverse or OFF   *****/
		/******   Allows the grabber to be on when shooter on   *****/
		grabber.grabberDirection(grabberDirection);


        /*****   Horizontal Conveyer Piston Deploy/Retract   *****/
		/*****   Grabber collecting balls always retract piston   *****/
		if (grabberDirection == Grabber.GrabberDirection.FORWARD) {
			conveyer.forwardingRetract();
		}
		//  Shooter shooting balls always retract piston
		else if (shooterEnable == true)  {
			conveyer.forwardingRetract();
		}
		//  Not grabbing or shooting balls allow piston deploy/retract
		else  {
	        if (horizontalPistonDeployRetract == true) {
				conveyer.changeForwardingState();
			}
		}

		
		/*****   Shooter Control   *****/
		if (shooterEnable == true) {
			if (hailMary == true) {
				shooter.autoShooterControl( Shooter.ShootLocation.HAIL_MARY );//manual
			}
			else if (trenchShot == true) {
				shooter.autoShooterControl( Shooter.ShootLocation.TRENCH );//manual
			}
			else {
				shooter.autoShooterControl(Shooter.ShootLocation.TEN_FOOT);//auto uses pid
			}
			
		}
		else  {
			shooter.manualShooterControl( Shooter.ShootLocation.OFF );
		}

		/*****   Conveyer Control   *****/
		// Can't have grabber & shooter on at same time
		if ((grabberDirection == Grabber.GrabberDirection.FORWARD)  &&
		    (shooterEnable    == false))  {
			conveyer.autoHorizontalControl();
			conveyer.autoVerticalControl();
			backupVerticalCount = 0;
		}
		else if ((grabberDirection == Grabber.GrabberDirection.REVERSE)  &&
        		 (shooterEnable    == false))  {
			conveyer.manualHorizontalControl(horizontalBeltState);
			conveyer.manualVerticalControl(verticalBeltState);
			backupVerticalCount = 0;
		}
		else if ((grabberDirection == Grabber.GrabberDirection.OFF)  &&
		         (shooterEnable    == true)) {
			// reverse the verticle conveyer to prevent the ball from 
			// jamming the shooter when it turns on
			if (backupVerticalCount < 5) {
				conveyer.manualHorizontalControl(Conveyer.ConveyerState.OFF);
				conveyer.autoVerticalDown();
				backupVerticalCount++;
			}
			else {
				// Waits for Shooter to Get Up to Speed
				//if ( shooter.shooterReady() == true ) {
				if (shooter.shooterReadyAuto() == true){
System.out.println("shooter ready");
					// Shooter at required RPM, Turn Conveyers On
					conveyer.manualHorizontalControl(Conveyer.ConveyerState.FORWARD);
					conveyer.manualVerticalControl(Conveyer.ConveyerState.FORWARD);
				}
				else {
System.out.println("shooter NOT ready");
					// Shooter below required RPM, Turn Conveyers Off
					conveyer.manualHorizontalControl(Conveyer.ConveyerState.OFF);
					conveyer.manualVerticalControl(Conveyer.ConveyerState.OFF);
				}
			}
		}
		else  {
			conveyer.manualHorizontalControl(Conveyer.ConveyerState.OFF);
			conveyer.manualVerticalControl(Conveyer.ConveyerState.OFF);
			backupVerticalCount = 0;
		}
	}




	/*************************************************
	 * 
	 * This code get run when robot is disabled
	 * Put code in here to record time when disabled
	 * 
	 *************************************************/
	public void
	disabledInit()
	{
		endTime = System.nanoTime();
		System.out.println("Elapsed Time: " + ((endTime - startTime)/1000000000.0));
		System.out.println("Disabled robot...");
		System.out.println("record time...");
	}

} //End of the Robot Class

