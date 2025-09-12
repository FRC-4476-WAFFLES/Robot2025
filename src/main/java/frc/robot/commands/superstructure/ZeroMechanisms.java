// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.GroundPivotConstants.GroundPivotPosition;
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
        RobotContainer.groundSuperstructure.pivot.zeroPivot();
        hasStartedPivot = false;

        RobotContainer.superstructure.elevator.applySetpoint(SuperstructureState.ZERO);
        
    }

    @Override
    public void execute() {
        if (RobotContainer.groundSuperstructure.pivot.isZeroing()) {
            return;
        } else {
            RobotContainer.groundSuperstructure.pivot.applySetpoint(GroundPivotPosition.ZEROING_CLEARANCE);
        }
        
        // Once elevator is done zeroing, ground intake is done, and out of the way, start pivot zeroing
        if (!RobotContainer.superstructure.elevator.isZeroing() && 
            !hasStartedPivot && 
            !RobotContainer.groundSuperstructure.pivot.isZeroing() &&
            RobotContainer.groundSuperstructure.pivot.atSetpoint()) {

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
            if (RobotContainer.groundSuperstructure.pivot.isZeroing()) {
                RobotContainer.groundSuperstructure.pivot.zeroPivot(); // Calling again cancels zeroing
            }
        }

        // elevatorTimer.stop();
    }

    @Override
    public boolean isFinished() {
        // Command is done when elevator is zeroed and pivot is zeroed
        return !RobotContainer.superstructure.elevator.isZeroing() && 
               hasStartedPivot && 
               !RobotContainer.superstructure.pivot.isZeroing();
    }
} 