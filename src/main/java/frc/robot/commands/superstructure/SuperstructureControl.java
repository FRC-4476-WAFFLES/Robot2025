// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.data.Constants.ManipulatorConstants.PivotPosition;
import frc.robot.subsystems.DynamicPathing;
import frc.robot.subsystems.DynamicPathing.DynamicPathingSituation;

/** Contains command factories that control the superstructure */
public class SuperstructureControl {
    // Private, hence cannot be instanciated
    private SuperstructureControl() {} 

    /**
     * A command that preemptively sets the elevator to L2 when in range of the reef to speed up scoring
     * @return The elevator's default command
     */
    public static Command ElevatorDefaultCommand() {
        return new FunctionalCommand(
            () -> {}, 
            () -> {
                if (RobotContainer.isOperatorOverride) {
                    return;
                }

                if (!DynamicPathing.isElevatorRetractionSafe()) {
                    // Do not move elevator down automatically until safely away from reef
                    return;
                }

                // Go to L2 automatically if in range to speed up motion
                if (RobotContainer.dynamicPathingSubsystem.getCurrentPathingSituation() == DynamicPathingSituation.REEF_CORAL) {
                    RobotContainer.elevatorSubsystem.setElevatorSetpoint(ElevatorLevel.L2);
                } else {
                    RobotContainer.elevatorSubsystem.setElevatorSetpoint(ElevatorLevel.REST_POSITION);
                }
            }, 
            (interrupted) -> {},
            () -> false, 
            RobotContainer.elevatorSubsystem
        );
    }

    /**
     * A command that preemptively sets the pivot to L2 when in range of the reef to speed up scoring
     * @return The pivot's default command
     */
    public static Command PivotDefaultCommand() {
        return new FunctionalCommand(
            () -> {}, 
            () -> {
                if (RobotContainer.isOperatorOverride) {
                    return;
                }
                
                // Go to L2 automatically if in range to speed up motion
                if (RobotContainer.dynamicPathingSubsystem.getCurrentPathingSituation() == DynamicPathingSituation.REEF_CORAL) {
                    RobotContainer.pivotSubsystem.setPivotPosition(PivotPosition.L2);
                } else {
                    RobotContainer.pivotSubsystem.setPivotPosition(PivotPosition.ZERO);
                }
            }, 
            (interrupted) -> {},
            () -> false, 
            RobotContainer.pivotSubsystem
        );
    }

    public static Command RestPositionCommand() {
        return new InstantCommand(() -> {
            RobotContainer.elevatorSubsystem.setElevatorSetpoint(ElevatorLevel.REST_POSITION);
            RobotContainer.pivotSubsystem.setPivotPosition(PivotPosition.ZERO);
        });
    }

    public static Command L4ScorePrepCommand() {
        return new FunctionalCommand(
            () -> {}, 
            () -> {
                if (RobotContainer.intakeSubsystem.isCoralLoaded()) {
                    RobotContainer.elevatorSubsystem.setElevatorSetpoint(ElevatorLevel.L2);
                    RobotContainer.pivotSubsystem.setPivotPosition(PivotPosition.ZERO);
                }
            }, 
            (interrupted) -> {
                if (interrupted) {
                    return;
                }
                RobotContainer.elevatorSubsystem.setElevatorSetpoint(ElevatorLevel.L4);
                RobotContainer.pivotSubsystem.setPivotPosition(PivotPosition.CLEARANCE_POSITION);
            },
            () -> DynamicPathing.isElevatorL4Ready(), 
            RobotContainer.elevatorSubsystem
        ).withTimeout(2);
    }
}
