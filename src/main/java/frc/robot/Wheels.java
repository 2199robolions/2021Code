package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpiutil.math.MathUtil;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.controller.PIDController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.networktables.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/***********************************************************************************************
 * 
 * 
Tuning Methods

Zeigler-Nichols tuning method works by increasing P until the system starts oscillating, 
and then using the period of the oscillation to calculate I and D.

    Start by setting I and D to 0.
	Increase P until the system starts oscillating for a period of Tu. You want the oscillation
	 to be large enough that you can time it. This maximum P will be referred to as Ku.
    Use the chart below to calculate different P, I, and D values.

Control Types 	P 	I 	D
P 	.5*Ku 	- 	-
PI 	.45*Ku 	.54*Ku/Tu 	-
PID 	.6*Ku 	1.2*Ku/Tu 	3*Ku*Tu/40

Note

The period of oscillation is one full ‘stroke’, there and back. 
Imagine a grandfather clock with a pendulum, when it is all the way to the right, swings to the left, 
and hits the right again, that is 1 period.

 * 
 ***********************************************************************************************/
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

	//static final double kToleranceDegrees = 1.0f;
	static final double kToleranceDegrees = 2.0f;
	static final double kLimeLightToleranceDegrees = 1.0f;

	//
	private boolean firstTime = true;
	private int count = 0;
	private double encoderTarget;

	//Limelight Variables
	private boolean limeLightFirstTime = true;
	private static final int ON_TARGET_COUNT = 20;
	private static final int ON_ANGLE_COUNT = 10;

	//Limelight
	private double m_LimelightCalculatedDistPrev = 0;
	public boolean limeControl = false;
	public int limeStatus = 0;
	public static final int LIMELIGHT_ON  = 0;
	public static final int LIMELIGHT_OFF = 1;
	private long timeOut;

	// Turn Controller
	//private static final double kP = 0.05;
	private static final double kP = 0.02;
	private static final double kI = 0.00;
	private static final double kD = 0.00;

	//Target Controller
	private int limeCount = 0;
	private PIDController targetController;
	private static final double tP = 0.05;
	private static final double tI = 0.20;
	private static final double tD = 0.00;
	private static final double tToleranceDegrees = 1.00f;

	//Circle variables & constants
	public static int revolutions;  
	private static double CIRCLE_POWER     = -0.60;   // this is the foward power
	private static double CIRCLE_ROTATION  = -0.47; // this is the clockwise rotation power
	//-0.60, -0.47 original
	//-0.80, -0.64 fast
	//-0.90, -0.76(4) super fast

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

	//Possible states of the circleTest function
	public static enum CircleRange {
		IN_RANGE,
		OUT_RANGE;
	}

	private CircleRange rangeState = CircleRange.OUT_RANGE;

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
		turnController.setIntegratorRange(-.25, .25); // do not change 
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
	public int forward(double feet, double heading){
		return forward(feet, heading, -0.6);
	}


	public int forward(double feet, double heading, double forwardPower) {
		double encoderCurrent;
		//double forwardPower = -.6;          // Negative Push Robot Forward
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

	public int forwardFullSpeed(double feet, double heading) {
		double encoderCurrent;
		//double forwardPower = -.6;          // Negative Push Robot Forward
		double rotationsPerFoot;
		double tempPower;
		double slowDistance = 3;

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
			tempPower = -1.00;
			//Target encoder values
			encoderTarget = encoderCurrent - (rotationsPerFoot * feet);

		}

		//Sets power to 1, unless the robot is within 3 feet of target, where it will be 0.6 power
		tempPower = -1.0;

		if(Math.abs(encoderTarget - encoderCurrent) < rotationsPerFoot * slowDistance){
			tempPower = -0.6;
		}

		if (feet < 0) {
			tempPower = tempPower * -1;
		}

		// Drive forward on a set oriantation
		drive.arcadeDrive( tempPower, turnController.calculate(ahrs.getYaw()) * -1 );

		// Current encoder values
		encoderCurrent = encoderLeft_1.getPosition();


		// Check runtime complete
		if (((encoderCurrent <= encoderTarget) && (feet >= 0)) ||
			((encoderCurrent >= encoderTarget) && (feet < 0))) {
			turnController.reset();
			drive.arcadeDrive( 0, 0);

			firstTime = true;
			System.out.println("Yaw: " + ahrs.getYaw());
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
		double pidOutput;
		long currentMs = System.currentTimeMillis();

		if (firstTime == true) {
			count = 0;
			firstTime = false;
			timeOut = currentMs + 2500;       // two.5 second time out
			turnController.setSetpoint(degrees);
		}
		
		if (currentMs > timeOut) {
			count = 0;
			firstTime = true;
			System.out.println("Timed out");
			return Robot.FAIL;
		}

		// Rotate
		pidOutput = turnController.calculate(ahrs.getYaw(), degrees);
		pidOutput = MathUtil.clamp(pidOutput, -0.75, 0.75);
		//System.out.println("Yaw: " + ahrs.getYaw());
		//System.out.println(pidOutput);
		drive.arcadeDrive( 0.0, pidOutput * -1, false);

		turnController.setTolerance(kToleranceDegrees);
		// CHECK: Routine Complete
		if (turnController.atSetpoint() == true) {
			count = count + 1;
			System.out.println("Count: " + count);

			if (count == ON_ANGLE_COUNT) {
				System.out.println("DONE");
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


	public int circleSlow(double exitAngle, boolean clockwise, boolean forwardDirection, int passes)   {
		CIRCLE_POWER = -0.6;
		CIRCLE_ROTATION = -0.47;
		return circle(exitAngle, clockwise, forwardDirection, passes);
	}
	public int circleMedium(double exitAngle, boolean clockwise, boolean forwardDirection, int passes)   {
		CIRCLE_POWER = -0.8;
		CIRCLE_ROTATION = -0.64;
		return circle(exitAngle, clockwise, forwardDirection, passes);
	}
	public int circleFast(double exitAngle, boolean clockwise, boolean forwardDirection, int passes)   {
		CIRCLE_POWER = -0.9;
		CIRCLE_ROTATION = -0.76;
		return circle(exitAngle, clockwise, forwardDirection, passes);
    }

	public int circle(double exitAngle, boolean clockwise, boolean forwardDirection, int passes)   {
		int range = 10;
		double maxRange = exitAngle + range;
		double minRange = exitAngle - range;

		double circleForward = 0;
		double circleRotate = 0;

		boolean edgeCase = false;
		
		//Stops the robot if it's turning for more than 5 seconds
		long currentMs = System.currentTimeMillis();

		if (firstTime == true) {
			revolutions = passes;
			timeOut = currentMs + 5000;
			firstTime = false;
			rangeState = CircleRange.OUT_RANGE;
		}
		if(currentMs > timeOut){

			drive.arcadeDrive( 0.0, 0.0 );
			firstTime = true;
			System.out.println("Timeout");
			rangeState = CircleRange.OUT_RANGE;
			return Robot.DONE;
		}

		if(exitAngle > 170 || exitAngle < -170){
			edgeCase = true;
		}

		//Bringing range windows into usable domain
		if(maxRange > 180)  {
			maxRange -= 360;
		} else if(maxRange < -180)  {
			maxRange += 360;
		} 

		if(minRange > 180)  {
			minRange -= 360;
		} else if(minRange < -180)  {
			minRange += 360;
		}
		//inputs of revolutions, exit angle, and the direction

		//Get angle of the robot for the yaw
		double yaw = ahrs.getYaw();
		
		//state machine that checks to see if a range around the exit angle is entered, and subtracts a revolution if yes
		//if revolutions is at 0, stop the robot, otherwise keep going
		if (clockwise == true) {
			circleRotate = CIRCLE_ROTATION;
		} 
		else if (clockwise == false) {
			circleRotate = CIRCLE_ROTATION * -1;
		}

		if (forwardDirection == true) {
			circleForward = CIRCLE_POWER;
		} 
		else if (forwardDirection == false) {
			circleForward = CIRCLE_POWER * -1;

		}
		drive.arcadeDrive(circleForward, circleRotate, false);

		if(rangeState == CircleRange.IN_RANGE) {
			if(revolutions == 0) {
				//System.out.println("Final Yaw: " + yaw);
				drive.arcadeDrive(0.0, 0.0);
				firstTime = true;
				System.out.println("Done: " + yaw + " Target: " + exitAngle + " min: " + minRange + " max: " + maxRange);
				rangeState = CircleRange.OUT_RANGE;
				return Robot.DONE;
			}
			
			if(edgeCase == false){
				if(minRange <= yaw && yaw <= maxRange){
					//do nothing
					return Robot.CONT;
				} else {
					revolutions--;
					rangeState = CircleRange.OUT_RANGE;
					return Robot.CONT;
				}
			} 
			else if(edgeCase == true){ //If it's around 180 degrees, it uses an OR instead of AND statement
				if(minRange <= yaw || yaw <= maxRange){
					//do nothing
					return Robot.CONT;
				} else {
					revolutions--;
					rangeState = CircleRange.OUT_RANGE;
					return Robot.CONT;
				}
			}
			
			

		} else if(rangeState == CircleRange.OUT_RANGE) {
				
			if(edgeCase == false){
				if(minRange <= yaw && yaw <= maxRange) {
					rangeState = CircleRange.IN_RANGE;
				} 
			}
			else if(edgeCase == true){
				if(minRange <= yaw || yaw <= maxRange) {
					rangeState = CircleRange.IN_RANGE;
				} 
			}
			return Robot.CONT;
		}	

		System.out.println("We shouldn't be here");
		rangeState = CircleRange.OUT_RANGE;
		return Robot.DONE;

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

	//0 degrees is straight forward
	//+- 180 is backwards
	//+ is to right (0 to +180)
	//- is to left (0 to -180)
	public void testYaw()  {
		System.out.println("Yaw " + ahrs.getYaw());
	}

	//Error = setPoint - yaw
	public void testPid(){
		turnController.setSetpoint(45);
		System.out.println(turnController.calculate(ahrs.getYaw()));
	}

	/* Is the PID error positive or negative? */
	public void testPidError()  {
		System.out.println("output:" + turnController.calculate(40, 50));
	}

	public void testArcadeRotation(){
		drive.arcadeDrive( 0.0, 0.25, false);
	}

	public void testRotation(double degrees){
		double pidOutput;

		turnController.setP(0.02);
		turnController.setI(0.0);
		turnController.setD(0.0);
		pidOutput = turnController.calculate(ahrs.getYaw(), degrees);
		pidOutput = MathUtil.clamp(pidOutput, -0.75, 0.75);
		System.out.println("Yaw: " + ahrs.getYaw() + " Pid Output: " + pidOutput);
		drive.arcadeDrive( 0.0, pidOutput * -1, false);
	}

	public void testCircleDrive(){
		drive.arcadeDrive(-0.6, -0.45, false);
	}
} //End of the Wheels Class 