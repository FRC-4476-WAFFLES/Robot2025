// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shark;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.SharkPivotConstants.SharkPivotPosition;

/** Add your docs here. */
public class SharkCommands {
    public static Command getIntakeCommand() {
        return new FunctionalCommand(
        () -> {
            RobotContainer.sharkPivot.setPivotPosition(SharkPivotPosition.DEPLOYED);
            RobotContainer.sharkIntake.setIntakeSpeed(8);
        }, 
        () -> {}, 
        (interrupted) -> {
            RobotContainer.isRunningL1Intake = false;
            
            RobotContainer.sharkIntake.setIntakeSpeed(0);

            if (!RobotContainer.sharkIntake.isCoralLoaded()) {
                // If we're just flipping back, use stowed
                RobotContainer.sharkPivot.setPivotPosition(SharkPivotPosition.STOWED);
            } else {
                RobotContainer.sharkPivot.setPivotPosition(SharkPivotPosition.L1);
            }
        },
        () -> RobotContainer.sharkIntake.isCoralLoaded(),
        RobotContainer.sharkPivot, RobotContainer.sharkIntake);
    }

    public static Command getOutakeCommand() {
        return new FunctionalCommand(
        () -> {
            RobotContainer.sharkPivot.setPivotPosition(SharkPivotPosition.L1);
            RobotContainer.sharkIntake.setIntakeSpeed(-2.4);
        }, 
        () -> {}, 
        (interrupted) -> {
            RobotContainer.sharkPivot.setPivotPosition(SharkPivotPosition.STOWED);
            RobotContainer.sharkIntake.setIntakeSpeed(0);
        },
        () -> !RobotContainer.sharkIntake.isCoralLoaded(),
        RobotContainer.sharkPivot, RobotContainer.sharkIntake);
    }
}
