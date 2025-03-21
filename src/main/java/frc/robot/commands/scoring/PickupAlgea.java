// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.intake.AlgeaIntake;
import frc.robot.commands.superstructure.ApplyScoringSetpoint;
import frc.robot.data.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.data.Constants.ManipulatorConstants.PivotPosition;
import frc.robot.data.Constants.ScoringConstants.ScoringLevel;
import frc.robot.subsystems.DynamicPathing;

public class PickupAlgea extends SequentialCommandGroup {
  /** Creates a new ScoreCoral. */
  private PickupAlgea(Command driveCommand, ScoringLevel scoringLevel, Command pathAwayCommand) {
    addCommands(
      new ParallelCommandGroup(
        RobotContainer.dynamicPathingSubsystem.wrapPathingCommand(driveCommand),
        // Deploy and pickup sequence
        new SequentialCommandGroup(
          new WaitUntilCommand(() -> DynamicPathing.isPastAlgaeClearancePoint()),
          new ParallelCommandGroup(
            new ApplyScoringSetpoint(scoringLevel),
            new AlgeaIntake()
          )
        )
      ),
      pathAwayCommand
    );
  }

  // public static boolean isOutOfReefRetractionDangerZone() {
  //   return DynamicPathingSubsystem.getDistanceToReef() > DynamicPathingSubsystem.REEF_SCORING_POSITION_OFFSET_ALGAE_CLEARANCE;
  // }

  public static Command pickupAlgeaWithPath(Command driveCommand, ScoringLevel scoringLevel, Command pathAwayCommand) {
    return new PickupAlgea(driveCommand, scoringLevel, pathAwayCommand).finallyDo((interruped) -> {
      if (interruped && !RobotContainer.intakeSubsystem.isAlgaeLoaded()) {
        // If interrupted, 
        RobotContainer.intakeSubsystem.setIntakeSpeed(-20);
      } else {
        RobotContainer.elevatorSubsystem.setElevatorSetpoint(ElevatorLevel.REST_POSITION);
        RobotContainer.pivotSubsystem.setPivotSetpoint(PivotPosition.CLEARANCE_POSITION);
      }
    });
  }
}
