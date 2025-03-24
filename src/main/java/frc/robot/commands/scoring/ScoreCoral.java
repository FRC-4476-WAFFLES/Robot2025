// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import java.util.Arrays;
import java.util.HashSet;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Controls;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.data.Constants.ManipulatorConstants.PivotPosition;
import frc.robot.data.Constants.ScoringConstants.ScoringLevel;
import frc.robot.subsystems.DynamicPathing;
import frc.robot.commands.AlignToPose;
import frc.robot.commands.intake.CoralOutake;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.scoring.PickupAlgae;

public class ScoreCoral extends SequentialCommandGroup {
  private static final double waitBeforeScoreL4 = 0.1;
  // All scoring commands require these subsystems
  public static final HashSet<Subsystem> commandRequirements = new HashSet<>(Arrays.asList(
    RobotContainer.driveSubsystem, 
    RobotContainer.pivotSubsystem, 
    RobotContainer.elevatorSubsystem, 
    RobotContainer.intakeSubsystem
  ));

  /* Timing variables */
  private final Timer totalScoringTimer = new Timer();
  private static final NetworkTable scoringTable = NetworkTableInstance.getDefault().getTable("ScoringMetrics");
  private static final DoublePublisher totalScoringTimePublisher = scoringTable.getDoubleTopic("Total ScoreCoral Duration").publish();

  /** Creates a new ScoreCoral. */
  private ScoreCoral(Command driveCommand, Pose2d finalAlignPose) {
    var pathingSubsystem = RobotContainer.dynamicPathingSubsystem;
    
    // Command to start timing
    Command startTimingCommand = new InstantCommand(() -> {
      totalScoringTimer.reset();
      totalScoringTimer.start();
      SmartDashboard.putBoolean("ScoreCoralInProgress", true);
    });
    
    // Command to end timing
    Command endTimingCommand = new InstantCommand(() -> {
      totalScoringTimer.stop();
      double finalScoringTime = totalScoringTimer.get();
      totalScoringTimePublisher.set(finalScoringTime);
      SmartDashboard.putNumber("Recent Total Scoring Time", finalScoringTime);
      SmartDashboard.putBoolean("ScoreCoralInProgress", false);
    });

    addCommands(
      // Start timing
      startTimingCommand,
      
      // Main scoring sequence
      new ParallelCommandGroup(
        pathingSubsystem.wrapPathingCommand(
          new SequentialCommandGroup(
            driveCommand,
            new AlignToPose(finalAlignPose)
          )
        ),
        new PrepareScoreCoral()
      ),
      
      // Wait until doNotScore is released
      new WaitUntilCommand(() -> !Controls.doNotScore.getAsBoolean()),
      
      // Outake the coral
      new CoralOutake(),
      
      // End timing
      endTimingCommand
    );
  }

  /* 
   * Abuse of command based programming. 
   * The constructor is private and can only be constructed by 
   * methods within the class that add decorators
   */
  
  /**
   * Scores coral given a path and a final target pose
   * @param driveCommand the pathing command
   * @param finalAlignPose the final pose (used for a final PID based alignment pass)
   * @return The command to score coral
   */
  public static Command scoreCoralWithPath(Command driveCommand, Pose2d finalAlignPose) {
    return new ScoreCoral(driveCommand, finalAlignPose).finallyDo(() -> {
      RobotContainer.elevatorSubsystem.setElevatorSetpoint(ElevatorLevel.REST_POSITION);

      if (RobotContainer.dynamicPathingSubsystem.getCoralScoringLevel() == ScoringLevel.L4) {
        RobotContainer.pivotSubsystem.setPivotPosition(PivotPosition.ZERO);
      } else {
        RobotContainer.pivotSubsystem.setPivotPosition(PivotPosition.CLEARANCE_POSITION);
      }
    });
  }

  /**
   * Scores coral given a path and a final target pose, and if the algae button is held,
   * automatically picks up algae after scoring.
   * @param driveCommand the pathing command
   * @param finalAlignPose the final pose (used for a final PID based alignment pass)
   * @return The command to score coral and optionally pick up algae after
   */
  public static Command scoreCoralWithPathAndAlgae(Command driveCommand, Pose2d finalAlignPose) {
    Command scoreCoralCommand = scoreCoralWithPath(driveCommand, finalAlignPose);
    
    return new SequentialCommandGroup(
      scoreCoralCommand,
      new DeferredCommand(() -> {
        // Check if the button is held down AND other conditions are met
        // (A variety of sanity checks)
        boolean shouldPickupAlgae = Controls.algaeAfterScoreButton.getAsBoolean() && 
          DynamicPathing.isRobotInRangeOfReefPathing() && 
          !RobotContainer.intakeSubsystem.isCoralLoaded() &&
          !RobotContainer.intakeSubsystem.isAlgaeLoaded() &&
          Controls.dynamicPathingButton.getAsBoolean();
        
        if (shouldPickupAlgae) {
          // Use the helper method in DynamicPathing to create the algae pickup command
          var dynamicPathing = RobotContainer.dynamicPathingSubsystem;
          Command algaeCommand = dynamicPathing.createAlgaePickupCommand();

          // createAlgaePickupCommand() is nullable
          if (algaeCommand != null) {
            return algaeCommand;
          }
        }
        // Return empty command otherwise
        return new InstantCommand();

      }, commandRequirements)
    );
  }
  
  /**
   * Scores coral given a final target pose. Performs no pathing of it's own.
   * @param finalAlignPose The final pose (used for a final PID based alignment pass)
   * @return The command to score coral
   */
  public static Command scoreCoralWithPose(Pose2d finalAlignPose) {
    return scoreCoralWithPath(new InstantCommand(), finalAlignPose);
  }

  /**
   * Scores coral from the nearest valid pose, based on the settings passed in. Performs no pathing of it's own.
   * Computes pose based on settings and current robot position. Drive to target before calling.
   * @param level scoringLevel for the desired level
   * @param rightSide scoring on the right or left side of the reef
   * @return The command to score coral
   */
  public static Command scoreCoralWithSettings(ScoringLevel level, boolean rightSide) {
    RobotContainer.dynamicPathingSubsystem.setCoralScoringLevel(level);
    RobotContainer.dynamicPathingSubsystem.setCoralScoringSide(rightSide);
    Pose2d targetCoralPose = RobotContainer.dynamicPathingSubsystem.getNearestCoralScoringLocation();

    return scoreCoralWithPose(targetCoralPose);
  }
}
