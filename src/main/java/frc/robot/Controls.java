package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.utils.WafflesUtilities;

/** 
 * Seperates controls a bit from RobotContainer while grouping controls specific constants together 
 */
public class Controls {
    // Replace with CommandPS4Controller or CommandJoystick if needed
    public static final CommandJoystick leftJoystick = new CommandJoystick(DriverConstants.leftJoystick);
    public static final CommandJoystick rightJoystick = new CommandJoystick(DriverConstants.rightJoystick);
    public static final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    // Constants
    private static final double JOYSTICK_DEADZONE_INNER = 0.025; // Below the inner value the input is zero
    private static final double JOYSTICK_DEADZONE_OUTER = 0.15; // Between the inner and outer value the input is interpolated towards it's actual value
    public static final double AXIS_DEADBAND = 0.1;  // Deadband for controller axes to prevent unintended activation
    public static final double MANUAL_ELEVATOR_CONTROL_MULTIPLIER = 2;  // Todo: Change this number

    /* Triggers */
    /* When triggers are referenced in multiple places, they are defined here to have a single source of truth */
    public static final Trigger dynamicPathingButton = rightJoystick.button(1);
    public static final Trigger algaeAfterScoreButton = leftJoystick.button(1);
    public static final Trigger doNotScore = Controls.operatorController.leftTrigger(Controls.AXIS_DEADBAND);
    public static final Trigger algaeOut = operatorController.rightBumper();

    public static class DriverConstants {
        public static final int leftJoystick = 0;
        public static final int rightJoystick = 1;
    }

    public static class OperatorConstants {
        public static final int kOperatorControllerPort = 2;
    }

    // Methods to get driver input
    public static double getDriveX() {
        double driveX = applyDeadzone(leftJoystick.getX());
        RobotContainer.telemetry.publishControlInfoX(driveX);
        return -driveX * PhysicalConstants.maxSpeed;
    }

    public static double getDriveY() {
        double driveY = applyDeadzone(leftJoystick.getY());
        RobotContainer.telemetry.publishControlInfoY(driveY);
        return -driveY * PhysicalConstants.maxSpeed;
    }

    public static Rotation2d getDriveRotation() {
        double driveRot = applyDeadzone(rightJoystick.getX());
        RobotContainer.telemetry.publishControlInfoRot(driveRot);
        return Rotation2d.fromRadians(-driveRot * PhysicalConstants.maxAngularSpeed);
    }

    // Smooths deadzone over range
    public static double applyDeadzone(double input){
        return Math.abs(input) > JOYSTICK_DEADZONE_OUTER ? 
        input : 
        (Math.abs(input) < JOYSTICK_DEADZONE_INNER ? 
            0 : 
            (WafflesUtilities.Lerp(0, input, WafflesUtilities.InvLerp(JOYSTICK_DEADZONE_INNER, JOYSTICK_DEADZONE_OUTER, input)) * Math.signum(input))
        );
    }

    // Clamps and squares input from two joysticks
    // Inputs should be in the 0-1 range
    public static Translation2d normalizeJoystickInput(double x, double y) {
        Translation2d inputVector = new Translation2d(x, y);
        double magnitude = inputVector.getNorm();

        if (magnitude > 1) {
            inputVector.div(magnitude);
            magnitude = 1;
        }
        
        // Square magnitude for more precise control
        inputVector.times(magnitude);

        return inputVector;
    }

    /* Methods to get operator input */
    public static double getOperatorRightY() {
        return operatorController.getRightY();
    }

    public static double getOperatorRightX() {
        return operatorController.getRightX();
    }
}
