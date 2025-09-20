// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.ManipulatorConstants;
import frc.robot.subsystems.DynamicPathing;
import frc.robot.subsystems.DynamicPathing.DynamicPathingSituation;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;

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
                    RobotContainer.superstructure.elevator.applySetpoint(SuperstructureState.HANDOFF_CLEAR);
                } else {
                    if (RobotContainer.intakeSubsystem.isAlgaeLoaded()) {
                        RobotContainer.superstructure.elevator.applySetpoint(SuperstructureState.ALGAE_REST);
                    } else {
                        if (RobotContainer.intakeSubsystem.isCoralLoaded()) {
                            RobotContainer.superstructure.elevator.applySetpoint(SuperstructureState.HANDOFF_CLEAR);   
                        } else {
                            RobotContainer.superstructure.elevator.applySetpoint(SuperstructureState.HANDOFF_READY);
                        }
                    }
                }
            }, 
            (interrupted) -> {},
            () -> false, 
            RobotContainer.superstructure.elevator
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
                    RobotContainer.superstructure.pivot.applySetpoint(SuperstructureState.HANDOFF_CLEAR);
                } else {
                    if (RobotContainer.intakeSubsystem.isCoralLoaded()) {
                        RobotContainer.superstructure.pivot.applySetpoint(SuperstructureState.HANDOFF_CLEAR);
                    } else {
                        if (RobotContainer.intakeSubsystem.isAlgaeLoaded()) {
                            RobotContainer.superstructure.pivot.applySetpoint(SuperstructureState.ALGAE_REST);
                        } else {
                            RobotContainer.superstructure.pivot.applySetpoint(SuperstructureState.HANDOFF_READY);
                        }
                    }
                }
            }, 
            (interrupted) -> {},
            () -> false, 
            RobotContainer.superstructure.pivot
        );
    }

    public static Command RestPositionCommand() {
        return new InstantCommand(() -> {
            RobotContainer.superstructure.applySuperstructureState(SuperstructureState.ZERO);
        });
    }

    public static Command L4ScorePrepCommand() {
        return new FunctionalCommand(
            () -> {}, 
            () -> {
                if (RobotContainer.intakeSubsystem.isCoralLoaded()) {
                    RobotContainer.superstructure.elevator.applySetpoint(SuperstructureState.HANDOFF_CLEAR);
                    RobotContainer.superstructure.pivot.applySetpoint(SuperstructureState.HANDOFF_CLEAR);
                }
            }, 
            (interrupted) -> {
                if (interrupted) {
                    return;
                }
                RobotContainer.superstructure.applySuperstructureState(SuperstructureState.L4);
            },
            () -> DynamicPathing.isElevatorL4Ready(), 
            RobotContainer.superstructure.elevator
        ).withTimeout(2);
    }
}
