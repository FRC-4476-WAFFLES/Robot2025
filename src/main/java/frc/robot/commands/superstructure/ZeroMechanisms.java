// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;

public class ZeroMechanisms extends Command {
    private boolean hasStartedPivot = false;
    // private Timer elevatorTimer = new Timer() ;

    public ZeroMechanisms() {
        addRequirements(RobotContainer.superstructure.requirements);
    }

    @Override
    public void initialize() {
        // Start elevator zeroing first
        RobotContainer.superstructure.elevator.zeroElevator();
        hasStartedPivot = false;

        RobotContainer.superstructure.elevator.applySetpoint(SuperstructureState.ZERO);

        RobotContainer.sharkPivot.zeroPivot();

        // elevatorTimer.reset();
        // elevatorTimer.start();
    }

    @Override
    public void execute() {
        // Once elevator is done zeroing, start pivot zeroing
        if (!RobotContainer.superstructure.elevator.isZeroing() && !hasStartedPivot) {
            RobotContainer.superstructure.pivot.zeroPivot();
            hasStartedPivot = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // If interrupted, make sure to stop both mechanisms
        if (interrupted) {
            if (RobotContainer.superstructure.elevator.isZeroing()) {
                RobotContainer.superstructure.elevator.zeroElevator(); // Calling again cancels zeroing
            }
            if (RobotContainer.superstructure.pivot.isZeroing()) {
                RobotContainer.superstructure.pivot.zeroPivot(); // Calling again cancels zeroing
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
        return !RobotContainer.superstructure.elevator.isZeroing() && 
               hasStartedPivot && 
               !RobotContainer.superstructure.pivot.isZeroing() &&
               !RobotContainer.sharkPivot.isZeroing();
    }
} 