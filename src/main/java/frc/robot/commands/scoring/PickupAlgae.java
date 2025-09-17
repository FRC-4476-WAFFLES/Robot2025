// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.AlignToPose;
import frc.robot.commands.intake.AlgaeIntake;
import frc.robot.commands.superstructure.ApplySuperstructureState;
import frc.robot.data.Constants.ManipulatorConstants;
import frc.robot.subsystems.DynamicPathing;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;

public class PickupAlgae extends SequentialCommandGroup {
  /** Creates a new ScoreCoral. */
  private PickupAlgae(Command driveCommand, SuperstructureState scoringLevel, Command pathAwayCommand, Pose2d pickupPose) {
    addCommands(
      new ParallelDeadlineGroup(
        // Deploy and pickup sequence
        new SequentialCommandGroup(
          // Move elevator first, since it's always safe to do so
          new InstantCommand(() -> {
            RobotContainer.superstructure.elevator.applySetpoint(scoringLevel);
            RobotContainer.superstructure.pivot.applySetpoint(ManipulatorConstants.PIVOT_CLEARANCE_POSITION);
            RobotContainer.intakeSubsystem.setIntakeSpeed(ManipulatorConstants.ALGAE_INTAKE_SPEED);
          }),
          // Wait until safe to move out pivot
          new WaitUntilCommand(() -> DynamicPathing.isPastAlgaeClearancePoint()),
          new ParallelDeadlineGroup(
            new AlgaeIntake(),
            new ApplySuperstructureState(scoringLevel)
          )
        ),

        // Move sequence
        RobotContainer.dynamicPathingSubsystem.wrapPathingCommand(
          new ParallelRaceGroup(
            new SequentialCommandGroup(
              driveCommand,
              new AlignToPose(pickupPose)
            ),
            new WaitUntilCommand(() -> RobotContainer.intakeSubsystem.isAlgaeLoaded())
          )
        )
      ),
      pathAwayCommand
    );
  }

  public static Command pickupAlgaeWithPath(Command driveCommand, SuperstructureState scoringLevel, Command pathAwayCommand, Pose2d pickupPose) {
    return new PickupAlgae(driveCommand, scoringLevel, pathAwayCommand, pickupPose).finallyDo((interruped) -> {
      RobotContainer.superstructure.elevator.applySetpoint(SuperstructureState.ZERO);
      RobotContainer.superstructure.pivot.applySetpoint(ManipulatorConstants.PIVOT_CLEARANCE_POSITION);
    });
  }
}
