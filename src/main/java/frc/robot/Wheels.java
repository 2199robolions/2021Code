package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.controller.PIDController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.networktables.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Wheels {
	// Led lights.
	private LedLights ledLights;

	// SPARK MAX
	private CANSparkMax sparkLeft_1;
	private CANSparkMax sparkLeft_2;
	private CANSparkMax sparkRight_1;
	private CANSparkMax sparkRight_2;

	//SPARK MAX ID's
	private static final int sparkRight_1_ID = 1;
	private static final int sparkRight_2_ID = 2;
	private static final int sparkLeft_1_ID  = 3;
	private static final int sparkLeft_2_ID  = 4;
	
	//Constants
	private final int WHEELS_CURRENT_LIMIT = 50;
	private final int FAIL_DELAY           = 5;
	private final double ROT_PER_FOOT_HIGH = 8.0;       // For High Gear
	private final double ROT_PER_FOOT_LOW  = 20.0;      // For LOW Gear

	//Variables
	double rightWheelAverage;
	double leftWheelAverage;
	int noTargetCount = 0;

	//ENCODERS
	private CANEncoder encoderLeft_1;
	private CANEncoder encoderLeft_2;
	private CANEncoder encoderRight_1;
	private CANEncoder encoderRight_2;

	//DifferentialDrive DRIVE
	private DifferentialDrive drive;
	
	//NAVX
	private AHRS ahrs;
	private PIDController turnController;

	static final double kToleranceDegrees = 1.0f;
	static final double kLimeLightToleranceDegrees = 1.0f;

	//
	private boolean firstTime = true;
	private int count = 0;
	private double encoderTarget;

	//Limelight Variables
	private boolean limeLightFirstTime = true;
	private static final int ON_TARGET_COUNT = 20;

	//Limelight
	private double m_LimelightCalculatedDistPrev = 0;
	public boolean limeControl = false;
	public int limeStatus = 0;
	public static final int LIMELIGHT_ON  = 0;
	public static final int LIMELIGHT_OFF = 1;
	private long timeOut;

	// Turn Controller
	private static final double kP = 0.05;
	private static final double kI = 0.00;
	private static final double kD = 0.00;

	//Target Controller
	private int limeCount = 0;
	private PIDController targetController;
	private static final double tP = 0.05;
	private static final double tI = 0.20;
	private static final double tD = 0.00;
	private static final double tToleranceDegrees = 1.00f;

	// Super Shifter pnuematics
	private DoubleSolenoid  superShifter;
	private final int SOLENOID_LOW_GEAR_ID      = 3;
	private final int SOLENOID_HIGH_GEAR_ID     = 4;
	private final int PCM_CAN_ID                = 0;

	// Possible States of the Super Shifter
	private enum ShifterState {
		HIGH,
		LOW;
	}
	private ShifterState shifterState;

	public static enum WheelMode {
		MANUAL,
		TARGET_LOCK;
	}

	public static enum TargetPipeline {
		TEN_FOOT,
		TRENCH;
	}

	/**
	 * CONSTRUCTOR
	 */
	public Wheels(LedLights led) {
		firstTime = true;
		limeLightFirstTime = true;
		ledLights = led;
		
		// SPARKS
		sparkLeft_1  = new CANSparkMax(sparkLeft_1_ID, MotorType.kBrushless);
		sparkLeft_2  = new CANSparkMax(sparkLeft_2_ID, MotorType.kBrushless);
		sparkRight_1 = new CANSparkMax(sparkRight_1_ID, MotorType.kBrushless);
		sparkRight_2 = new CANSparkMax(sparkRight_2_ID, MotorType.kBrushless);        

		//Encoders
		encoderLeft_1  = new CANEncoder(sparkLeft_1);
		encoderLeft_2  = new CANEncoder(sparkLeft_2);
		encoderRight_1 = new CANEncoder(sparkRight_1);
		encoderRight_2 = new CANEncoder(sparkRight_2);

		// 2 motors in each supershifter
		// Do NOT use speedControllerGroup for sparkMax's here
		sparkLeft_1.follow( sparkLeft_2, false);
		sparkRight_1.follow(sparkRight_2, false);

		// Spark Current Limit
		sparkLeft_1.setSmartCurrentLimit(WHEELS_CURRENT_LIMIT);
		sparkLeft_2.setSmartCurrentLimit(WHEELS_CURRENT_LIMIT);
		sparkRight_1.setSmartCurrentLimit(WHEELS_CURRENT_LIMIT);
		sparkRight_2.setSmartCurrentLimit(WHEELS_CURRENT_LIMIT);

		// DifferentialDrive DRIVE
		drive = new DifferentialDrive(sparkLeft_2, sparkRight_2);

		// Make sure wheels start in off state
		drive.arcadeDrive(0, 0);

		//Configure Super Shifter gear
		superShifter = new DoubleSolenoid(PCM_CAN_ID, SOLENOID_LOW_GEAR_ID, SOLENOID_HIGH_GEAR_ID);
		superShifter.set(Value.kForward);
		shifterState = ShifterState.HIGH;

		//NAVX
		try {
			ahrs = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			System.out.println("Error Instantiating navX MXP: " + ex.getMessage());
		}

		ahrs.reset();

		while (ahrs.isConnected() == false) {
			// System.out.println("Connecting navX");
		}
		System.out.println("navX Connected");

		while (ahrs.isCalibrating() == true) {
			System.out.println("Calibrating navX");
		}
		System.out.println("navx Ready");

		// At Start, Set navX to ZERO
		ahrs.zeroYaw();

		//PID Controllers
		turnController = new PIDController(kP, kI, kD);
		targetController = new PIDController(tP, tI, tD);
		
		/* Max/Min input values.  Inputs are continuous/circle */
		turnController.enableContinuousInput(-180.0, 180.0);
		targetController.enableContinuousInput(-27.0, 27.0);

		/* Max/Min output values */
		//Turn Controller
		turnController.setIntegratorRange(-.35, .35); // do not change 
		turnController.setTolerance(kToleranceDegrees);

		//Target Controller
		targetController.setIntegratorRange(-.2, .2); // do not change 
		targetController.setTolerance(kLimeLightToleranceDegrees);

		//Variables
		rightWheelAverage = (encoderRight_1.getVelocity() + encoderRight_2.getVelocity()) / 2;
		leftWheelAverage = (encoderLeft_1.getVelocity() + encoderLeft_2.getVelocity()) / 2;

		/**
		 * Limelight Modes
		 */
		//Force the LED's to off to start the match
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
		//Set limelight mode to vision processor
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
		//Sets limelight streaming mode to Standard (The primary camera and the secondary camera are displayed side by side)
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
		//Sets limelight pipeline to 0 (light off)
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
	}

	/*
	 * CONTROLLER DRIVE
	 */
	public void controllerDrive(double forward, double clockwiseRot) {
		drive.arcadeDrive(-1 * forward, -1 * clockwiseRot);
	}

	/*
	 * Autonomous Forward
	 */
	public int forward(double feet, double heading) {
		double encoderCurrent;
		double forwardPower = -.6;          // Negative Push Robot Forward
		double rotationsPerFoot;

		// Determines Feet for the Specific Gear Ratio
		if (shifterState == ShifterState.LOW) {
			rotationsPerFoot = ROT_PER_FOOT_LOW;
		} else if (shifterState == ShifterState.HIGH) {
			rotationsPerFoot = ROT_PER_FOOT_HIGH;
		} else {
			rotationsPerFoot = 0;
		}

		// current encoder values
		encoderCurrent = encoderLeft_1.getPosition();

		if (firstTime == true) {
			firstTime = false;
			turnController.setSetpoint(heading);

			//Target encoder values
			encoderTarget = encoderCurrent - (rotationsPerFoot * feet);

		}

		if (feet < 0) {
			forwardPower = forwardPower * -1;
		}

		// Drive forward on a set oriantation
		drive.arcadeDrive( forwardPower, turnController.calculate(ahrs.getYaw()) * -1 );

		// Current encoder values
		encoderCurrent = encoderLeft_1.getPosition();


		// Check runtime complete
		if (((encoderCurrent <= encoderTarget) && (feet >= 0)) ||
			((encoderCurrent >= encoderTarget) && (feet < 0))) {
			turnController.reset();
			drive.arcadeDrive( 0, 0);

			firstTime = true;

			return Robot.DONE; 
		} 
		else {
			return Robot.CONT;
		}
	}

	/*
	 * Autonomous Rotate
	 */
	public int rotate(double degrees){
		long currentMs = System.currentTimeMillis();

		if (firstTime == true) {
			count = 0;
			firstTime = false;
			timeOut = currentMs + 2000;       // two second time out
			turnController.setSetpoint(degrees);
		}
		
		if (currentMs > timeOut) {
			count = 0;
			firstTime = true;
			return Robot.FAIL;
		}

		// Rotate
		drive.arcadeDrive( 0.0, turnController.calculate(ahrs.getYaw(), degrees) * -1 );

		turnController.setTolerance(kToleranceDegrees);
		// CHECK: Routine Complete
		if (turnController.atSetpoint() == true) {
			count = count + 1;

			if (count == ON_TARGET_COUNT) {
				turnController.reset();
				count = 0;
				firstTime = true;
				drive.arcadeDrive( 0.0, 0.0 );
				return Robot.DONE;
			} else {
				return Robot.CONT;
			}

		}
		else {    
			count = 0;
			return Robot.CONT;
		}
	}

	/**
	 * LIMELIGHT TARGETING
	 */
	public int limelightPIDTargeting( TargetPipeline pipeline) {
		double m_LimelightCalculatedDist = 0;
		long currentMs = System.currentTimeMillis();

		if (limeLightFirstTime == true) {
			targetController.setSetpoint(0.0);
			changeLimelightLED(LIMELIGHT_ON);
			ledLights.limelightAdjusting();
			timeOut = currentMs + 2000;       // two second time out
			limeLightFirstTime = false;
			// System.out.println("TimeOut " + timeOut);
		}

		// Whether the limelight has any valid targets (0 or 1)
		double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
		// Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees) [54 degree tolerance]
		double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
		// Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees) [41 degree tolerance]
		//double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
		// Target Area (0% of image to 100% of image) [Basic way to determine distance]
		// Use lidar for more acurate readings in future
		//double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

		if (tv < 1.0) {
			ledLights.limelightNoValidTarget();
			drive.arcadeDrive( 0.0, 0.0 );

			noTargetCount++;

			if (noTargetCount <= FAIL_DELAY) {
				return Robot.CONT;
			}
			else {
				System.out.println("No Valid target");

				noTargetCount = 0;
				targetController.reset();
				limeCount = 0;
				limeLightFirstTime = true;
				drive.arcadeDrive( 0.0, 0.0 );
				ledLights.limelightNoValidTarget();
				return Robot.FAIL;
			}
		}
		else {
			noTargetCount = 0;
			ledLights.limelightAdjusting();
		}

		// Rotate
		m_LimelightCalculatedDist = targetController.calculate(tx, 0.0);
		drive.arcadeDrive( 0.0, m_LimelightCalculatedDist);

		// CHECK: Routine Complete
		if (targetController.atSetpoint() == true) {
			limeCount++;
		} 
		else if (m_LimelightCalculatedDistPrev != 0.0 && 
				Math.abs(m_LimelightCalculatedDist - m_LimelightCalculatedDistPrev) <= 0.01) {
			limeCount++;
		} 

		if (limeCount >= ON_TARGET_COUNT) {
			targetController.reset();
			limeCount = 0;
			limeLightFirstTime = true;
			drive.arcadeDrive( 0.0, 0.0 );
			ledLights.limelightFinished();

			return Robot.DONE;
		}

		m_LimelightCalculatedDistPrev = m_LimelightCalculatedDist;

		// limelight time out readjust
		if (currentMs > timeOut) {
			System.out.println("timeout " + tx + " Target Acquired " + tv);
			targetController.reset();
			limeCount = 0;
			limeLightFirstTime = true;
			drive.arcadeDrive( 0.0, 0.0 );
			ledLights.limelightNoValidTarget();            
			return Robot.FAIL;
		}

		return Robot.CONT;   
	}

	/**
	 * GEAR SHIFT
	 */
	public void gearShift() {
		
		// Change the State of the Piston
		if (shifterState == ShifterState.LOW) {
			superShifter.set(Value.kForward);
			shifterState = ShifterState.HIGH;
		}
		else if (shifterState == ShifterState.HIGH) {
			superShifter.set(Value.kReverse);
			shifterState = ShifterState.LOW;
		}
	}

	private static final double CameraMountingAngle = 22.0;	// 25.6 degrees
	private static final double CameraHeightFeet 	= 26.5 / 12;	        // 16.5 inches
	private static final double VisionTapeHeightFt 	= 7 + (7.5 / 12.0) ;	// 8ft 2.25 inches
	
	private static double mountingRadians = Math.toRadians(CameraMountingAngle); // a1, converted to radians

	// find result of h2 - h1
	private static double differenceOfHeights = VisionTapeHeightFt - CameraHeightFeet;
	
	/** 
	 * D = (h2 - h1) / tan(a1 + a2). This equation, along with known numbers, helps find the distance
	 * from a target.
	 */
	public double getDistance() {
	  // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees) [41 degree tolerance]
	  double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
		
	  // a2, converted to radians
	  double radiansToTarget = Math.toRadians(ty); 

	  // find result of a1 + a2
	  double angleInRadians = mountingRadians + radiansToTarget;

	  // find the tangent of a1 + a2
	  double tangentOfAngle = Math.tan(angleInRadians); 

	  // Divide the two results ((h2 - h1) / tan(a1 + a2)) for the distance to target
	  double distance = differenceOfHeights / tangentOfAngle;

	  // outputs the distance calculated
	  return distance; 
	}

	/** 
	 * a1 = arctan((h2 - h1) / d - tan(a2)). This equation, with a known distance input, helps find the 
	 * mounted camera angle.
	 */
	public double getCameraMountingAngle(double measuredDistance) {
	  // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees) [41 degree tolerance]
	  double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

	  // convert a2 to radians
	  double radiansToTarget = Math.toRadians(ty);

	  // find result of (h2 - h1) / d
	  double heightOverDistance = differenceOfHeights / measuredDistance;

	  // find result of tan(a2)
	  double tangentOfAngle = Math.tan(radiansToTarget);

	  // (h2-h1)/d - tan(a2) subtract two results for the tangent of the two sides
	  double TangentOfSides = heightOverDistance - tangentOfAngle; 

	  // invert tangent operation to get the camera mounting angle in radians
	  double newMountingRadians = Math.atan(TangentOfSides);

	  // change result into degrees
	  double cameraMountingAngle = Math.toDegrees(newMountingRadians);
	  
	  return cameraMountingAngle; // output result
	}

	/**
	 * Change Limelight Modes
	 */
	// Changes Limelight Pipeline
	public void changeLimelightPipeline(int pipeline) {
		// Limelight Pipeline
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
	}
	
	// Change Limelight LED's
	public void changeLimelightLED(int mode) {
		// if mode = 0 limelight on : mode = 1 limelight off
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(mode);
	}

	/**
	 * DEBUG / TEST FUNCTIONS
	 */

	public void enableDriveMotors( String side ) {
		if (side == "left" || side == "Left") {
			sparkLeft_2.set(1.00);
		}

		if (side == "right" || side == "Right") {
			sparkRight_2.set(1.00);
		}
	}

	public void stopDriveMotors( String side ) {
		if (side == "left" || side == "Left") {
			sparkLeft_2.set(0.00);
		}

		if (side == "right" || side == "Right") {
			sparkRight_2.set(0.00);
		}
	}

	public void testEncoder()  {
		System.out.println("encoder: " +  encoderLeft_1.getPosition() );
	}

	public void testYaw()  {
		System.out.println("Yaw " + ahrs.getYaw());
	}

	public void testPid(){
		turnController.setSetpoint(0);
		System.out.println(turnController.calculate(ahrs.getYaw()));
	}

} //End of the Wheels Class 