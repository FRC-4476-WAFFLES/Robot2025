// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.ElevatorConstants;
import frc.robot.data.Constants.ManipulatorConstants.PivotPosition;
import frc.robot.data.Constants.ScoringConstants.ScoringLevel;
import frc.robot.subsystems.DynamicPathing;
import frc.robot.subsystems.Elevator;

/* Continuously adjusts position of elevator and pivot to desired scoring level */
public class PrepareScoreCoral extends Command {
  private final Elevator elevatorSubsystem = RobotContainer.elevatorSubsystem;

  /** Creates a new PrepareCoralScore. */
  public PrepareScoreCoral() {
    addRequirements(RobotContainer.pivotSubsystem, RobotContainer.elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var scoringLevel = RobotContainer.dynamicPathingSubsystem.getCoralScoringLevel();
    if (scoringLevel == ScoringLevel.L1) {
      if (DynamicPathing.getDistanceToReef() < DynamicPathing.REEF_MIN_SCORING_DISTANCE_L1) {
        return;
      }
    }

    if (scoringLevel == ScoringLevel.L4) {
      if (DynamicPathing.isElevatorL4Ready()) {
        RobotContainer.elevatorSubsystem.setElevatorSetpoint(scoringLevel.getElevatorLevel());
        if (elevatorSubsystem.getElevatorPositionMeters() > ElevatorConstants.PIVOT_L4_CLEAR_HEIGHT_MAX) {
          RobotContainer.pivotSubsystem.setPivotPosition(scoringLevel.getPivotPosition());
        } else {
          RobotContainer.pivotSubsystem.setPivotPosition(PivotPosition.CLEARANCE_POSITION);
        }
      } else {
        RobotContainer.pivotSubsystem.setPivotPosition(PivotPosition.ZERO);
      }
      return;
    }

    RobotContainer.elevatorSubsystem.setElevatorSetpoint(scoringLevel.getElevatorLevel());
    RobotContainer.pivotSubsystem.setPivotPosition(scoringLevel.getPivotPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
