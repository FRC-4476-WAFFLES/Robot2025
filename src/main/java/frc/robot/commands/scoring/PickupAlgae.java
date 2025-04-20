// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.AlignToPose;
import frc.robot.commands.intake.AlgaeIntake;
import frc.robot.commands.superstructure.ApplyScoringSetpoint;
import frc.robot.data.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.data.Constants.ManipulatorConstants;
import frc.robot.data.Constants.ManipulatorConstants.PivotPosition;
import frc.robot.data.Constants.ScoringConstants.ScoringLevel;
import frc.robot.subsystems.DynamicPathing;

public class PickupAlgae extends SequentialCommandGroup {
  /** Creates a new ScoreCoral. */
  private PickupAlgae(Command driveCommand, ScoringLevel scoringLevel, Command pathAwayCommand, Pose2d pickupPose) {
    addCommands(
      new ParallelDeadlineGroup(
        // Deploy and pickup sequence
        new SequentialCommandGroup(
          // Move elevator first, since it's always safe to do so
          new InstantCommand(() -> {
            RobotContainer.elevatorSubsystem.setElevatorSetpoint(scoringLevel.getElevatorLevel());
            RobotContainer.pivotSubsystem.setPivotPosition(PivotPosition.CLEARANCE_POSITION);
            RobotContainer.intakeSubsystem.setIntakeSpeed(ManipulatorConstants.ALGAE_INTAKE_SPEED);
          }),
          // Wait until safe to move out pivot
          new WaitUntilCommand(() -> DynamicPathing.isPastAlgaeClearancePoint()),
          new ParallelCommandGroup(
            new ApplyScoringSetpoint(scoringLevel),
            new AlgaeIntake()
          )
        ),

        // Move sequence
        RobotContainer.dynamicPathingSubsystem.wrapPathingCommand(
          new SequentialCommandGroup(
            driveCommand,
            new AlignToPose(pickupPose)
          )
        )
      ),
      pathAwayCommand
    );
  }

  public static Command pickupAlgaeWithPath(Command driveCommand, ScoringLevel scoringLevel, Command pathAwayCommand, Pose2d pickupPose) {
    return new PickupAlgae(driveCommand, scoringLevel, pathAwayCommand, pickupPose).finallyDo((interruped) -> {
      RobotContainer.elevatorSubsystem.setElevatorSetpoint(ElevatorLevel.REST_POSITION);
      RobotContainer.pivotSubsystem.setPivotPosition(PivotPosition.CLEARANCE_POSITION);
    });
  }
}
