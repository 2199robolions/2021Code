package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;

// Variables
	/**
	 * At 0.6 Power RPM is 3240
	 * At 0.69 Power RPM is 3834 MAX
	 * At 0.7 Power RPM is 3750
	 * At 0.8 Power RPM is 4220
	 * At 0.9 Power RPM is 4770
	 * At 1.0 Power RPM is 5240 (5400)
	 */

	 /****************************************
	  * 
	  *  Data from March 10, 2020
	  *
	  power		rpm
	  .60		3420
	  .65		3680
	  .70		3942
	  .75		4200
	  .80		4451
	  .85		4697
	  .90		4942
	  .95		5182
	  1.00		5422

	  ruler			limelight		power
	  distance		distance		to make shot
	  10							.65
	  11							.65
	  12							.65
	  13							.65
	  14							
	  15							
	  16							
	  17							.58
	  18							
	  19			17.5			.60
	  20			18.3			.62 .65
	  21			18.4			.65
	  22			18.9			.68
	  23			19.7			.70
	  24			20.4			.70
	  25			20.9			.70

	  7/17/2021
	  squishy balls shoot low, stiff ones shoot high
	  */

public class Shooter {
	
	// SPARK MAX
	private CANSparkMax shooter_1;
	private CANSparkMax shooter_2;

	// SPARK MAX ID's
	private int shooter_1_ID = 5;
	private int shooter_2_ID = 6;

	// Encoders
	private CANEncoder encoder_Shooter_1;
	private CANEncoder encoder_Shooter_2;

	// CONSTANTS
	public final double OFF_POWER       = 0.00;
	public final double TEN_FOOT_POWER  = 0.67; //According to test, should give 4600 rpm by default
	public final double TRENCH_POWER    = 0.69; //.648 //.645
	public final double HAIL_MARY_POWER = 0.71;

	public final double MAX_TARGET_RPM       = 5300; 
	public final double OFF_TARGET_RPM       = 0;
	private final double ERROR_TARGET_RPM    = 150.0;
	public final double TEN_FOOT_TARGET_RPM  = 3830; //3720
	public final double TRENCH_TARGET_RPM    = 3970; //3662
	public final double HAIL_MARY_TARGET_RPM = 4040; //5375

	
	public double targetVelocity; //was private
	private double targetPower;
	private int targetCount = 0;

	public static enum ShootLocation {
		OFF,
		TEN_FOOT,
		TRENCH,
		HAIL_MARY;
	}

	// Shooter PID Controller
	private PIDController shooterController;

	private final double kToleranceDegrees = 2.0f;

	private static final double kP = 0.0004; //0.0003 old value
	private static final double kI = 0.00;
	private static final double kD = 0.00;
	
	/**
	 * CONSTRUCTOR
	 */
	public Shooter() {
		// SPARK Max
		shooter_1 = new CANSparkMax(shooter_1_ID, MotorType.kBrushless);
		shooter_2 = new CANSparkMax(shooter_2_ID, MotorType.kBrushless);
		
		// Set Shooter to off to Start the Match
		shooter_1.set(0.0);
		shooter_2.set(0.0);

		// Encoders
		encoder_Shooter_1 = shooter_1.getEncoder();
		encoder_Shooter_2 = shooter_2.getEncoder();

		shooterController = new PIDController(kP, kI, kD);
	  //  shooterController.enableContinuousInput(0.0, 5500.0);
	  //  shooterController.setIntegratorRange(0.0, 1.0);
	  //  shooterController.setTolerance(50.0);
	}


	public void autoShooterControl(ShootLocation location) {
		double  powerError;
		double  power;

		if (location == ShootLocation.OFF) {
			powerError = OFF_POWER;
			targetVelocity = OFF_TARGET_RPM;
			targetPower    = OFF_POWER;
		}
		else if (location == ShootLocation.TEN_FOOT) {
			powerError = shooterController.calculate( encoder_Shooter_1.getVelocity(), TEN_FOOT_TARGET_RPM);
			targetVelocity = TEN_FOOT_TARGET_RPM;
			targetPower    = TEN_FOOT_POWER;
		}
		else if (location == ShootLocation.TRENCH) {
			powerError = shooterController.calculate( encoder_Shooter_1.getVelocity(), TRENCH_TARGET_RPM);
			System.out.println("Trench Shot");
			targetVelocity = TRENCH_TARGET_RPM;
			targetPower    = TRENCH_POWER;
		}
		else if (location == ShootLocation.HAIL_MARY) {
			powerError = shooterController.calculate( encoder_Shooter_1.getVelocity(), HAIL_MARY_TARGET_RPM);
			targetVelocity = HAIL_MARY_TARGET_RPM;
			targetPower    = HAIL_MARY_POWER;
		}
		else {
			powerError = OFF_POWER;
			targetVelocity = OFF_TARGET_RPM;
			targetPower    = OFF_POWER;
		}

		power = MathUtil.clamp(targetPower + powerError, 0.0, 1.0);
		//System.out.println("power:" + power + " rpm:" + encoder_Shooter_1.getVelocity());
		SmartDashboard.putNumber("power", power);
		SmartDashboard.putNumber("rpm", encoder_Shooter_1.getVelocity());

		shooter_1.set(power);
		shooter_2.set(power * -1); 
	}

	public void manualShooterControl(ShootLocation location) {

		if (location == ShootLocation.OFF) {
			shooter_1.set(OFF_POWER);
			shooter_2.set(OFF_POWER); 
			targetVelocity = OFF_TARGET_RPM;
		}
		else if (location == ShootLocation.TEN_FOOT) {
			shooter_1.set(TEN_FOOT_POWER);
			shooter_2.set(TEN_FOOT_POWER * -1);
			targetVelocity = TEN_FOOT_TARGET_RPM;
		}
		else if (location == ShootLocation.TRENCH) {
			shooter_1.set(TRENCH_POWER);
			shooter_2.set(TRENCH_POWER * -1);
			targetVelocity = TRENCH_TARGET_RPM;
		}
		else if (location == ShootLocation.HAIL_MARY) {
			shooter_1.set(HAIL_MARY_POWER);
			shooter_2.set(HAIL_MARY_POWER * -1);
			targetVelocity = HAIL_MARY_TARGET_RPM;
		}
		else {
			shooter_1.set(OFF_POWER);
			shooter_2.set(OFF_POWER);
			targetVelocity = OFF_TARGET_RPM;
		}
	}

	public boolean shooterReadyAuto() {
		double rpm;

		rpm = encoder_Shooter_1.getVelocity();
		//System.out.println("RPM: " + rpm);
		if ( (rpm > (targetVelocity - ERROR_TARGET_RPM)) &&
			 (rpm < (targetVelocity + ERROR_TARGET_RPM)) )  {
			targetCount ++;
			if(targetCount >= 5){ //10 old value
				return true;
			} 
			else{
				return false;
			}
		}
		else {
			targetCount = 0;
			return false;
		}
	}

	public boolean shooterReady() {

	//System.out.println("velocity:" + encoder_Shooter_1.getVelocity() + " tgtVelocity:" + targetVelocity);
		if (encoder_Shooter_1.getVelocity() >= targetVelocity) {
			return true;
		}
		else {
			return false;
		}
	}

	public void testShoooter(double power) {
		shooter_1.set(power);
		shooter_2.set(power * -1);

		System.out.println("Power: " + power + " RPM: " + encoder_Shooter_1.getVelocity());
	}

	/**
	 * DEBUG
	 */
	public void printSpeed() {
		double π = Math.PI;
		double wheel_size = 4;                                                                  // Wheel diameter Inches 

		double RPM = (encoder_Shooter_1.getVelocity() + (encoder_Shooter_2.getVelocity() * -1) ) / 2;   // Rotations per minute average
		double RPH = RPM / 60;                                                                  // Rotations per hour
		
		double circumferenceInches = wheel_size * π;                                            // Circumference in Inches
		double circumferenceFeet = circumferenceInches / 12;                                    // Circumference in Feet
		double circumferenceMiles = circumferenceFeet / 5280;                                   // Circumference in Miles

		double MPH = RPH * circumferenceMiles;                                                  // Miles Per Hour

		if (RPM > 0) { 
			//System.out.println("MPH: " + MPH);
			if (MPH != 0) {
				System.out.println("RPM 1: " + encoder_Shooter_1.getVelocity());
				System.out.println("RPM 2: " + encoder_Shooter_2.getVelocity());
			}
		}
	}

	public void enableShooterFullPower() {
		shooter_1.set(0.70);
		shooter_2.set(-0.70);
		System.out.println("RPM 1: " + encoder_Shooter_1.getVelocity());
	}

	public void enableShooterMotor1() {
		shooter_1.set(0.50);
	}

	public void enableShooterMotor2() {
		shooter_2.set(-0.50);
	}

	public void stopShooterMotor1() {
		shooter_1.set(0.00);
	}

	public void stopShooterMotor2() {
		shooter_2.set(0.00);
	}

} //End of the Shooter Class