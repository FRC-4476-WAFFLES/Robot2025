// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.semiauto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.RobotContainer;
import frc.robot.commands.AlgeaIntake;
import frc.robot.data.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.data.Constants.ManipulatorConstants.PivotPosition;

public class PickupAlgea extends SequentialCommandGroup {
  /** Creates a new ScoreCoral. */
  private PickupAlgea(Command driveCommand) {
    addCommands(
      new ParallelCommandGroup(
        driveCommand,
        new PrepareScoreAlgea()
      ),
      new AlgeaIntake()
    );
  }

  public static Command pickupAlgeaWithPath(Command driveCommand) {
    return new PickupAlgea(driveCommand).finallyDo(() -> {
      RobotContainer.elevatorSubsystem.setElevatorSetpoint(ElevatorLevel.REST_POSITION);
      RobotContainer.pivotSubsystem.setPivotSetpoint(PivotPosition.CLEARANCE_POSITION);
    });
  }
}
