// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import java.util.Arrays;
import java.util.HashSet;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Controls;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.ManipulatorConstants.PivotPosition;
import frc.robot.data.Constants.ScoringConstants.ScoringLevel;
import frc.robot.subsystems.DynamicPathing;
import frc.robot.commands.AlignToPose;
import frc.robot.commands.intake.CoralOutake;

public class ScoreCoral extends SequentialCommandGroup {
  /** A collection of scoring parameters */
  public record CoralScoringParameters(
    double maxVelocity,
    Rotation2d maxThetaVelocity,

    double maxDistanceX,
    double maxDistanceY,
    Rotation2d maxThetaDifference
  ) {}

  public static final CoralScoringParameters L4Params = new CoralScoringParameters(
    0.05, 
    Rotation2d.fromDegrees(1), 
    0.05, 
    0.03, 
    Rotation2d.fromDegrees(2)
  );

  public static final CoralScoringParameters L3Params = new CoralScoringParameters(
    0.03, 
    Rotation2d.fromDegrees(1), 
    0.03, 
    0.03, 
    Rotation2d.fromDegrees(2)
  );

  public static final CoralScoringParameters L2Params = new CoralScoringParameters(
    0.03, 
    Rotation2d.fromDegrees(1), 
    0.03, 
    0.03, 
    Rotation2d.fromDegrees(2)
  );

  // Avoids super early releases
  public static final double PIVOT_DEADBAND_L4 = 24; 

  /* Timing variables */
  private final Timer totalScoringTimer = new Timer();
  private static final NetworkTable scoringTable = NetworkTableInstance.getDefault().getTable("ScoringMetrics");
  private static final DoublePublisher totalScoringTimePublisher = scoringTable.getDoubleTopic("Total ScoreCoral Duration").publish();

  /** Creates a new ScoreCoral. */
  private ScoreCoral(Command driveCommand, Pose2d finalAlignPose, double maxSpeed) {
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

    // Pick the parameter set for the current level
    CoralScoringParameters chosenParameters;
    if (RobotContainer.dynamicPathingSubsystem.getCoralScoringLevel() == ScoringLevel.L4) {
      chosenParameters = L4Params;
    } else if (RobotContainer.dynamicPathingSubsystem.getCoralScoringLevel() == ScoringLevel.L3) {
      chosenParameters = L3Params;
    } else {
      chosenParameters = L2Params;
    }
    

    // Once this becomes true, release coral
    Trigger scoreTrigger = new Trigger(() -> {
      Pose2d errorPose = RobotContainer.driveSubsystem.getRobotPose().relativeTo(finalAlignPose);
      ChassisSpeeds currentSpeeds = RobotContainer.driveSubsystem.getRobotChassisSpeeds();
      double velocityMagnitude = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
      
      boolean poseValid = 
        Math.abs(errorPose.getX()) <= chosenParameters.maxDistanceX &&
        Math.abs(errorPose.getY()) <= chosenParameters.maxDistanceY && 
        Math.abs(errorPose.getRotation().getDegrees()) <= chosenParameters.maxThetaDifference.getDegrees();
      boolean velocityValid = 
        velocityMagnitude <= chosenParameters.maxVelocity &&
        currentSpeeds.omegaRadiansPerSecond <= chosenParameters.maxThetaVelocity.getRadians();

      // Only for L4
      boolean pivotValidL4 = true;
      if (RobotContainer.dynamicPathingSubsystem.getCoralScoringLevel() == ScoringLevel.L4) {
        pivotValidL4 = Math.abs(RobotContainer.pivotSubsystem.getPivotPosition() - PivotPosition.L4.getDegrees()) < PIVOT_DEADBAND_L4; 
      }

      return poseValid && velocityValid && pivotValidL4;
    });

    addCommands(
      // Start timing
      startTimingCommand,
      
      // Main scoring sequence
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          // Wait until doNotScore is released
          new WaitUntilCommand(() -> !Controls.doNotScore.getAsBoolean() && scoreTrigger.getAsBoolean()),
          
          // Outake the coral
          new CoralOutake()
        ),
        pathingSubsystem.wrapPathingCommand(
          new AlignToPose(finalAlignPose)
        ),
        new PrepareScoreCoral()
      ),
      
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
   * @param maxSpeed the maximum speed for the final PID based alignment
   * @return The command to score coral
   */
  public static Command scoreCoralWithPath(Command driveCommand, Pose2d finalAlignPose, double maxSpeed) {
    return new ScoreCoral(driveCommand, finalAlignPose, maxSpeed).finallyDo(() -> {
      if (RobotContainer.dynamicPathingSubsystem.getCoralScoringLevel() == ScoringLevel.L4) {
        RobotContainer.pivotSubsystem.setPivotPosition(PivotPosition.ZERO);
        // Do not lower elevator after L4 score
      } else {
        RobotContainer.pivotSubsystem.setPivotPosition(PivotPosition.CLEARANCE_POSITION);
        // RobotContainer.elevatorSubsystem.setElevatorSetpoint(ElevatorLevel.REST_POSITION);
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
  public static Command scoreCoralWithPathAndAlgae(Command driveCommand, Pose2d finalAlignPose, double maxSpeed) {
    Command scoreCoralCommand = scoreCoralWithPath(driveCommand, finalAlignPose, maxSpeed);
    
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

      }, DynamicPathing.actionCommandRequirements)
    );
  }
  
  /**
   * Scores coral given a final target pose. Performs no pathing of it's own.
   * @param finalAlignPose The final pose (used for a final PID based alignment pass)
   * @return The command to score coral
   */
  public static Command scoreCoralWithPose(Pose2d finalAlignPose) {
    return scoreCoralWithPath(new InstantCommand(), finalAlignPose, Double.MAX_VALUE);
  }

  /**
   * Scores coral from the nearest valid pose, based on the settings passed in. Performs no pathing of it's own.
   * Computes pose based on settings and current robot position. Drive to target before calling.
   * @param level scoringLevel for the desired level
   * @param rightSide scoring on the right or left side of the reef
   * @return The command to score coral
   */
  public static Command scoreCoralWithSettings(ScoringLevel level, boolean rightSide) {
    if (!RobotContainer.intakeSubsystem.isCoralLoaded()) {
      return new InstantCommand();
    }
    RobotContainer.dynamicPathingSubsystem.setCoralScoringLevel(level);
    RobotContainer.dynamicPathingSubsystem.setCoralScoringSide(rightSide);
    Pose2d targetCoralPose = RobotContainer.dynamicPathingSubsystem.getNearestCoralScoringLocation();

    return scoreCoralWithPath(new InstantCommand(), targetCoralPose, DynamicPathing.CORAL_PATH_END_SPEED);
  }
}
