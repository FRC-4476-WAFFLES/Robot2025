// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.ElevatorConstants.ElevatorLevel;

public class ZeroMechanisms extends Command {
    private boolean hasStartedPivot = false;
    // private Timer elevatorTimer = new Timer() ;

    public ZeroMechanisms() {
        addRequirements(RobotContainer.elevatorSubsystem, RobotContainer.pivotSubsystem);
    }

    @Override
    public void initialize() {
        // Start elevator zeroing first
        RobotContainer.elevatorSubsystem.zeroElevator();
        hasStartedPivot = false;

        RobotContainer.elevatorSubsystem.setElevatorSetpoint(ElevatorLevel.REST_POSITION);

        RobotContainer.sharkPivot.zeroPivot();

        // elevatorTimer.reset();
        // elevatorTimer.start();
    }

    @Override
    public void execute() {
        // Once elevator is done zeroing, start pivot zeroing
        if (!RobotContainer.elevatorSubsystem.isZeroing() && !hasStartedPivot) {
            RobotContainer.pivotSubsystem.zeroPivot();
            hasStartedPivot = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // If interrupted, make sure to stop both mechanisms
        if (interrupted) {
            if (RobotContainer.elevatorSubsystem.isZeroing()) {
                RobotContainer.elevatorSubsystem.zeroElevator(); // Calling again cancels zeroing
            }
            if (RobotContainer.pivotSubsystem.isZeroing()) {
                RobotContainer.pivotSubsystem.zeroPivot(); // Calling again cancels zeroing
            }
            if (RobotContainer.sharkPivot.isZeroing()) {
                RobotContainer.sharkPivot.zeroPivot(); // Calling again cancels zeroing
            }
        }

        // elevatorTimer.stop();
    }

    @Override
    public boolean isFinished() {
        // Command is done when elevator is zeroed and pivot is zeroed
        return !RobotContainer.elevatorSubsystem.isZeroing() && 
               hasStartedPivot && 
               !RobotContainer.pivotSubsystem.isZeroing() &&
               !RobotContainer.sharkPivot.isZeroing();
    }
} 