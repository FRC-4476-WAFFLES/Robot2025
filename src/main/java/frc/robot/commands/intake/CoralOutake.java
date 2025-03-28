// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.RobotContainer.intakeSubsystem;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotContainer;


public class CoralOutake extends Command {
  public static final double OUTTAKE_POSITION_CHANGE = 6; // rotations
  
  private double outtakeEndPosition = 0;
  /** Creates a new CoralIntake. */
  public CoralOutake() {
    addRequirements(RobotContainer.intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Make sure the intake doesn't detect us as having loaded algae in this motion
    RobotContainer.intakeSubsystem.setNoAlgaeFlag(true);
    outtakeEndPosition = intakeSubsystem.getCurrentPosition() - OUTTAKE_POSITION_CHANGE;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var scoringLevel = RobotContainer.dynamicPathingSubsystem.getCoralScoringLevel();
    switch (scoringLevel) {
      case L1:
        RobotContainer.intakeSubsystem.setIntakeSpeed(8.0); // Reverse to spit out backwards
        break;

      case L4:
        RobotContainer.intakeSubsystem.setIntakeSpeed(-80.0); // Fast to ensure fitting on post
        break;

      default:
        RobotContainer.intakeSubsystem.setIntakeSpeed(-16.0); // Slower to avoid bouncing off L2-L3
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeSubsystem.setIntakeSpeed(0);
    RobotContainer.intakeSubsystem.setNoAlgaeFlag(false);
    
    // Get timing information from NetworkTables
    double alignmentTime = NetworkTableInstance.getDefault()
        .getTable("ScoringMetrics")
        .getEntry("FinalAlignCoral Duration")
        .getDouble(0.0);
    
    double totalScoringTime = NetworkTableInstance.getDefault()
        .getTable("ScoringMetrics")
        .getEntry("Total ScoreCoral Duration")
        .getDouble(0.0);
    
    // Record timing metrics without tracking success/failure
    RobotContainer.telemetry.recordScoringTime(alignmentTime, totalScoringTime);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeSubsystem.getCurrentPosition() <= outtakeEndPosition;

    // if (DriverStation.isAutonomous()) {
    //   return timer.get() > 0;
    // }
    // return timer.get() > 0.2;
  }
}
