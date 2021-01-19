package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Controller {
    
    // DEADBAND CONSTANT
    //private final double DEADBAND = .02;

    // CONTROLLER OBJECTS
    Joystick joystick;
    XboxController xbox;

    /*
     * CONSTRUCTOR
     */
    public Controller() {
        joystick = new Joystick(1);
        xbox     = new XboxController(0);
    }

    /*********************************************************
     * 
     * Joystick Functions
     * 
     ********************************************************/
    // Joystick Forward Returns a Negative Value. Multiply by -1 to Return a Positive Value
    public double getForwardPower() {
        return joystick.getY() * -1;
    }

    // Joystick Clockwise Rotation Returns a Positive Value.
    public double getClockwiseRotation() {
        return joystick.getZ();
    }

    public boolean hailMary() {
        return joystick.getRawButton(2);
    }

    public boolean enableTargetLock() {
        return joystick.getRawButtonPressed(3);
    }

    public boolean shiftGears() {
        return joystick.getRawButtonPressed(6);
    }

    public boolean enableShooter() {
        return joystick.getTrigger();
    }

    public boolean enableTrenchShot() {
        return joystick.getRawButton(4);
    }

    /*********************************************************
     * 
     * Xbox Controllor Functions
     * 
     ********************************************************/
    // Start Button Pressed
    public boolean autoKill() {
        return xbox.getStartButtonPressed();
    }

    // A button pressed
    public boolean colorRotate() {
        return xbox.getAButtonPressed();
    }

    // B Shooter pistion State
    public boolean bButtonPressed() {
        return xbox.getBButtonPressed();
    }

    // X Deploys Color Sensor
    public boolean colorDeployRetract() {
        return xbox.getXButtonPressed();
    }

    // Y Deploys / Retracts Grabber
    public boolean grabberDeployRetract() {
        return xbox.getYButtonPressed();
    }

    // Right Bumper Pressed
    public boolean climberAllArmsUp() {
        return xbox.getBumper(Hand.kRight);
    }

    // Left Bumper Pressed
    public boolean climberTopArmDown() {
        return xbox.getBumper(Hand.kLeft);
    }

    // Left Trigger
    public double colorWheelManualControl() {
        return xbox.getTriggerAxis(Hand.kLeft);
    }

    // Right Trigger
    public double getClimberPower() {
        double power;
        power = xbox.getTriggerAxis(Hand.kRight);

        //trigger dead band
        if (power > 0.1) {
            return power;
        }
        else {
            return 0;
         }
    }

    // Upper left, Up, and Upper right on the DPad returns forward
    // Lower left, Down, and Lower right on the DPad returns reverse 
    public Grabber.GrabberDirection getGrabberDir() {
        int selection = xbox.getPOV();

        if ( (selection == 315) || (selection == 0) || (selection == 45) ) {
            return Grabber.GrabberDirection.FORWARD;
        }
        else if ( (selection == 225) || (selection == 180) || (selection == 135) ) {
            return Grabber.GrabberDirection.REVERSE;
        }
        else {
            return Grabber.GrabberDirection.OFF;
        }

    }

    // Forward on the stick returns positive, backwards returns negative
    public Conveyer.ConveyerState getHorizonalBeltState() {
        double xboxY = xbox.getY(Hand.kRight) * -1;

        if (xboxY >= 0.2) {
            return Conveyer.ConveyerState.FORWARD;
        }
        else if (xboxY <= -0.2) {
            return Conveyer.ConveyerState.REVERSE;
        }
        else {
            return Conveyer.ConveyerState.OFF;
        }
    }

    // Forward on the stick returns positive, backwards returns negative
    public Conveyer.ConveyerState getVerticalBeltState() {
        double xboxY = xbox.getY(Hand.kLeft) * -1;

        if (xboxY >= 0.2) {
            return Conveyer.ConveyerState.FORWARD;
        }
        else if (xboxY <= -0.2) {
            return Conveyer.ConveyerState.REVERSE;
        }
        else {
            return Conveyer.ConveyerState.OFF;
        }
    }

    // Xbox Sticks Pressed
    public boolean getForwardingPressed() {
        boolean rightStick = xbox.getStickButtonPressed(Hand.kRight);
        boolean leftStick  = xbox.getStickButtonPressed(Hand.kLeft );

        if (rightStick || leftStick) {
            return true;
        }
        else {
            return false;
        }
    }

    /*
     * DEADBAND & REFINING CONTROL
     */
    /*
    private double modify(double power) {

       // DEADBAND
       if ((power < DEADBAND) && (power > DEADBAND * -1)) {
           return 0;
       }

       // Refining Control: Cubing Values
       if (power > 0) {
           power = power * power * power;
       } 
       else if (power < 0) {
           power = power * power * power;
       } 
       else {
           power = 0;
       }

       return power;
    }
    */

}// end of Controller Class