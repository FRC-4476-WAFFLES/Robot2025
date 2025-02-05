package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.data.Constants.PhysicalConstants;

/** 
 * Seperates controls a bit from RobotContainer while grouping controls specific constants together 
 */
public class Controls {
    // Replace with CommandPS4Controller or CommandJoystick if needed
    public static final CommandJoystick leftJoystick = new CommandJoystick(DriverConstants.leftJoystick);
    public static final CommandJoystick rightJoystick = new CommandJoystick(DriverConstants.rightJoystick);
    public static final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    public static final double MANUAL_ELEVATOR_CONTROL_MULTIPLIER = 2;  // Todo: Change this number
    public static final double AXIS_DEADBAND = 0.1;  // Deadband for controller axes to prevent unintended activation

    public static class DriverConstants {
        public static final int leftJoystick = 0;
        public static final int rightJoystick = 1;
    }

    public static class OperatorConstants {
        public static final int kOperatorControllerPort = 2;
    }

    // Methods to get driver input
    public static double getDriveX() {
        return -leftJoystick.getX() * PhysicalConstants.maxSpeed;
    }

    public static double getDriveY() {
        return -leftJoystick.getY() * PhysicalConstants.maxSpeed;
    }

    public static Rotation2d getDriveRotation() {
        return Rotation2d.fromRadians(-rightJoystick.getX() * PhysicalConstants.maxAngularSpeed);
    }

    /* Methods to get operator input */
    public static double getOperatorRightY() {
        return operatorController.getRightY();
    }

    public static double getOperatorRightX() {
        return operatorController.getRightX();
    }
}
