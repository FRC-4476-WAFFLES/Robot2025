// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;

public class ZeroMechanisms extends Command {
    private boolean hasStartedPivot = false;

    public ZeroMechanisms() {
        addRequirements(RobotContainer.elevatorSubsystem, RobotContainer.pivotSubsystem);
    }

    @Override
    public void initialize() {
        // Start elevator zeroing first
        RobotContainer.elevatorSubsystem.zeroElevator();
        hasStartedPivot = false;
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
        }
    }

    @Override
    public boolean isFinished() {
        // Command is done when elevator is zeroed and pivot is zeroed
        return !RobotContainer.elevatorSubsystem.isZeroing() && 
               hasStartedPivot && 
               !RobotContainer.pivotSubsystem.isZeroing();
    }
} 