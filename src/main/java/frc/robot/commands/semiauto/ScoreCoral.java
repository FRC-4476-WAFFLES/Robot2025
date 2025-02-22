// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.semiauto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.CoralOutake;
import frc.robot.data.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.data.Constants.ManipulatorConstants.PivotPosition;
import frc.robot.data.Constants.ScoringConstants.ScoringLevel;

public class ScoreCoral extends SequentialCommandGroup {
  private static final double waitBeforeScore = 0.25;

  /** Creates a new ScoreCoral. */
  private ScoreCoral(Command driveCommand) {
    addCommands(
      new ParallelCommandGroup(
        driveCommand,
        new PrepareScoreCoral()
      ),
      // Wait until doNotScore is released
      new WaitCommand(waitBeforeScore).onlyIf(() -> RobotContainer.dynamicPathingSubsystem.getCoralScoringLevel() == ScoringLevel.L4),
      new WaitUntilCommand(() -> !RobotContainer.doNotScore.getAsBoolean()),
      new CoralOutake()
    );
  }

  public static Command scoreCoralWithPath(Command driveCommand) {
    return new ScoreCoral(driveCommand).finallyDo(() -> {
      RobotContainer.elevatorSubsystem.setElevatorSetpoint(ElevatorLevel.REST_POSITION);
      if (RobotContainer.dynamicPathingSubsystem.getCoralScoringLevel() == ScoringLevel.L4) {
        RobotContainer.pivotSubsystem.setPivotSetpoint(PivotPosition.ZERO);
      } else {
        RobotContainer.pivotSubsystem.setPivotSetpoint(PivotPosition.CLEARANCE_POSITION);
      }
      
    });
  }
}
