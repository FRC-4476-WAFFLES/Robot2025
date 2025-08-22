// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GroundIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.GroundPivotConstants.GroundPivotPosition;

/** Add your docs here. */
public class GroundIntakeCommands {
    public static Command getIntakeCommand() {
        return new FunctionalCommand(
        () -> {
            RobotContainer.groundPivot.setPivotPosition(GroundPivotPosition.DEPLOYED);
            RobotContainer.groundIntake.setIntakeSpeed(8);
        }, 
        () -> {}, 
        (interrupted) -> {
            RobotContainer.isRunningL1Intake = false;
            
            RobotContainer.groundIntake.setIntakeSpeed(0);

            if (!RobotContainer.groundIntake.isCoralLoaded()) {
                // If we're just flipping back, use stowed
                RobotContainer.groundPivot.setPivotPosition(GroundPivotPosition.STOWED);
            } else {
                RobotContainer.groundPivot.setPivotPosition(GroundPivotPosition.L1);
            }
        },
        () -> RobotContainer.groundIntake.isCoralLoaded(),
        RobotContainer.groundPivot, RobotContainer.groundIntake);
    }

    public static Command getOutakeCommand() {
        return new FunctionalCommand(
        () -> {
            RobotContainer.groundPivot.setPivotPosition(GroundPivotPosition.L1);
            RobotContainer.groundIntake.setIntakeSpeed(-2.4);
        }, 
        () -> {}, 
        (interrupted) -> {
            RobotContainer.groundPivot.setPivotPosition(GroundPivotPosition.STOWED);
            RobotContainer.groundIntake.setIntakeSpeed(0);
        },
        () -> !RobotContainer.groundIntake.isCoralLoaded(),
        RobotContainer.groundPivot, RobotContainer.groundIntake);
    }
}
