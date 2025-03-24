// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import static frc.robot.RobotContainer.dynamicPathingSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.data.Constants;
import frc.robot.data.Constants.ScoringConstants;
import frc.robot.data.Constants.ScoringConstants.ScoringLevel;
import frc.robot.subsystems.DynamicPathing;

/* Continuously adjusts position of elevator and pivot to desired scoring level */
public class PrepareScoreCoral extends Command {
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
    RobotContainer.elevatorSubsystem.setElevatorSetpoint(scoringLevel.getElevatorLevel());
    RobotContainer.pivotSubsystem.setPivotPosition(scoringLevel.getPivotPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.pivotSubsystem.isPivotAtSetpoint() &&
           RobotContainer.elevatorSubsystem.isElevatorAtSetpoint() &&
           !RobotContainer.dynamicPathingSubsystem.isPathing(); // Ends only once pathing is done
  }
}
