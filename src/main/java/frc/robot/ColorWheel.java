/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import java.util.HashMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.Servo;




/**
 * docs here.
 */
public class ColorWheel {
    // SPARK MAX
    private CANSparkMax sparkWheel;

    // SPARK MAX ID's
    private final int SPARK_WHEEL_ID = 12;

    //Constants
    private final int SPINNER_CURRENT_LIMIT = 20;
    private final double WHEEL_RATIO = 15.00;               //18:1 cirumference ratio from NEO to Color Wheel
    private final double CLICKS_PER_NEO = 9.87;             //clicks per NEO revolution
    private final double TARGET_WHEEL_REVS = 4;             //# of times we want to rotate the color wheel
    private final double TARGET_CLICKS = CLICKS_PER_NEO * WHEEL_RATIO * TARGET_WHEEL_REVS;    //total
    private final int COLOR_SENSOR_PWM_PORT  = 1;

    private final double WHEEL_SPEED = 0.4;

    //ENCODERS
    private CANEncoder wheelEncoder;

    // SOLENOID CHANNELS
    private DoubleSolenoid motorDeploy;

    private final int DEPLOY_ID         = 3;
    private final int RETRACT_ID        = 2;
    private final int PCM_CAN_ID        = 20;

    // Enumerator for color wheel Piston States
    public static enum MotorState {
        DEPLOY,
        RETRACT;
    }
    private MotorState motorState;

    //private HashMap colorMap = new HashMap<String, String>();
    //private HashMap colorEnum = new HashMap<String, Integer>();
    
    // Variables
    public Color detectedColor;
    public String colorString = "";
    public ColorMatchResult match;
    private boolean firstTime = true;
    private double encoderTarget;
    private double encoderCurrent;
    private String targetColor;
    private Servo colorWheelActuator;


    /**
     * Change the I2C port below to match the connection of your color sensor
     */
    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    /**
     * A Rev Color Sensor V3 object is constructed with an I2C port as a 
     * parameter. The device will be automatically initialized with default 
     * parameters.
     */
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    /**
     * A Rev Color Match object is used to register and detect known colors. This can 
     * be calibrated ahead of time or during operation.
     * 
     * This object uses a simple euclidian distance to estimate the closest match
     * with given confidence range.
     */
    private final ColorMatch m_colorMatcher = new ColorMatch();
    
    //variables
    private double redValue = 0;
    private double blueValue = 0;
    private double greenValue = 0;

    private String gameData;
    
    /** 
     * Constructor
     */
    public ColorWheel() {
        // Set Color Wheel Motor to retracted
        motorDeploy = new DoubleSolenoid(PCM_CAN_ID, RETRACT_ID, DEPLOY_ID);
        motorDeploy.set(Value.kReverse);
        motorState = MotorState.RETRACT;

        // SPARKS
        sparkWheel = new CANSparkMax(SPARK_WHEEL_ID, MotorType.kBrushless);

        // ENCODER
        wheelEncoder = new CANEncoder(sparkWheel);

        // Current Limit
        sparkWheel.setSmartCurrentLimit(SPINNER_CURRENT_LIMIT);

        // Set Motor to 0
        sparkWheel.set(0.0);

        // Linear Actuator
        colorWheelActuator = new Servo(COLOR_SENSOR_PWM_PORT);
        colorWheelActuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

        // Bring Actuator Down
        colorSensorRetract();

        // Variables
        firstTime = true;
    }

    /** 
     * Color Sensor Calibrate
     */
    public void calibrate() {
        // placeholder
    }

    /** 
     * Color Sensor Deploy / Retract
     */
    public void deployRetract()  {
        if (motorState == MotorState.RETRACT)  {
            motorDeploy.set(Value.kForward);
            motorState = MotorState.DEPLOY;
            colorSensorDeploy();
        }
        else if (motorState == MotorState.DEPLOY)  {
            motorDeploy.set(Value.kReverse);
            motorState = MotorState.RETRACT;
            colorSensorRetract();
        }
    }

    /**
     * Get the Raw Sensor
     */
    public Color getColor() {
  
        Color color = m_colorSensor.getColor();
  
        redValue   = Double.parseDouble(String.valueOf(color.red));
        blueValue  = Double.parseDouble(String.valueOf(color.blue));
        greenValue = Double.parseDouble(String.valueOf(color.green));
        
        return color;
    }

    /**
     * Get Current Color
     */
    public String colorMatch() {
        // Run the color match algorithm on our detected color
        match = m_colorMatcher.matchClosestColor(getColor());
        
        // figure out the color based on vlaues and display it
        if ((redValue >= 0.17) && (redValue <= 0.21) && 
            (greenValue >= 0.43) && (greenValue <= 0.47) && 
            (blueValue >= 0.33) && (blueValue <= 0.37)) {
                colorString = "Blue";
        }
        else if ((redValue >= 0.31) && (redValue <= 0.35) && 
                (greenValue >= 0.41) && (greenValue <= 0.45) && 
                (blueValue >= 0.20) && (blueValue <= 0.24)) {
                    colorString = "Red";
        }
        else if ((redValue >= 0.20) && (redValue <= 0.24) && 
                (greenValue >= 0.49) && (greenValue <= 0.53) && 
                (blueValue >= 0.24) && (blueValue <= 0.28)) {
                    colorString = "Green";
        }
        else if ((redValue >= 0.26) && (redValue <= 0.30) && 
                (greenValue >= 0.50) && (greenValue <= 0.54) && 
                (blueValue >= 0.17) && (blueValue <= 0.21)) {
                    colorString = "Yellow";
        }
        else {
            colorString = "Unknown";
        }

        System.out.println(colorString);

        return colorString;
    }

    /**
     * Manually run the colorwheel
     */
    public void rotateColorWheel(double power) {
        sparkWheel.set(power);
    }

    /**
     * Code to make the color wheel spin 4 times
     */
    public int colorWheelSpin() {
        
        encoderCurrent = wheelEncoder.getPosition();

        //Target encoder values
        if (firstTime == true) {
            encoderTarget = encoderCurrent + TARGET_CLICKS;

            firstTime = false;
        }

        // turn color wheel
        sparkWheel.set(WHEEL_SPEED);

        // Print Motor Position
        System.out.println("Wheel Encoder: " + encoderCurrent);

        // Check Color Wheel 
        if (encoderCurrent >= encoderTarget) {
            sparkWheel.set(0);
            firstTime = true;

            return Robot.DONE; 
        } 
        else {
            return Robot.CONT; 
        }
    }

    /**
     * Spin to the Target Color for Final Spin
     */
    public int colorWheelColorMatch() {
        
        if (firstTime == true) {
            targetColor = getTargetColor();
            SmartDashboard.putString("Target Color", targetColor);

            firstTime = false;
        }

        // Turn Color Wheel
        sparkWheel.set(WHEEL_SPEED);

        // Check color wheel 
        if (colorMatch() == targetColor) {
            sparkWheel.set(0);

            return Robot.DONE; 
        }
        else {
            firstTime = true;

            return Robot.CONT; 
        }       
    }

    /**
     * Get color given for Final Spin
     */
    private String getTargetColor() {
        
        // Code for getting colors from field
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        String targetColorString = "";

        if(gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'B' :
                    //Blue case code
                    System.out.println("Spin to Red.");
                    targetColorString = "Red";
                    break;
                case 'G' :
                    //Green case code
                    System.out.println("Spin to Yellow.");
                    targetColorString = "Yellow";
                    break;
                case 'R' :
                    //Red case code
                    System.out.println("Spin to Blue.");
                    targetColorString = "Blue";
                    break;
                case 'Y' :
                    //Yellow case code
                    System.out.println("Spin to Green.");
                    targetColorString = "Green";
                    break;
                default :
                    //This is corrupt data
                    System.out.println("Corrupt Data!");
                    targetColorString = "Corrupt";
                    break;
            }
        }
        else {
            //Code for no data received yet
            System.out.println("No color recieved yet.");
        }

        return targetColorString;
    }

    public void seeInSuffleboard (Shuffleboard board) {
        /**
         * Open Smart Dashboard or Shuffleboard to see the color detected by the 
         * sensor.
         */
        colorMatch(); // update detected color and match confidence
        System.out.println("Color:\t\t" + detectedColor);
        System.out.println("Confidence:\t" + match.confidence);
    }

    public void colorSensorDeploy() {
        colorWheelActuator.setSpeed(1.0);
    }

    public void colorSensorRetract() {
        colorWheelActuator.setSpeed(-1.0);
    }

} // End of the ColorWheel Class