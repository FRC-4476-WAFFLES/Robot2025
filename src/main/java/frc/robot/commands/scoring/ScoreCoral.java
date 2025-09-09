// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Controls;
import frc.robot.RobotContainer;
import frc.robot.commands.AlignToPose;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.intake.CoralOutake;
import frc.robot.commands.superstructure.ApplySuperstructureState;
import frc.robot.data.Constants.ScoringConstants;
import frc.robot.data.Constants.ScoringConstants.CoralScoringParameters;
import frc.robot.subsystems.DynamicPathing;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.utils.WafflesUtilities;

public class ScoreCoral extends SequentialCommandGroup {
  public static final double SCORING_FINISHED_DISTANCE = DynamicPathing.REEF_CORAL_CLEAR_DISTANCE + 0.2; 
  
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
    if (RobotContainer.dynamicPathingSubsystem.getCoralScoringLevel() == SuperstructureState.L4) {
      chosenParameters = ScoringConstants.L4Params;
    } else if (RobotContainer.dynamicPathingSubsystem.getCoralScoringLevel() == SuperstructureState.L3) {
      chosenParameters = ScoringConstants.L3Params;
    } else {
      chosenParameters = ScoringConstants.L2Params;
    }
    

    // Once this becomes true, release coral
    Trigger scoreTrigger = new Trigger(() -> {
      Pose2d errorPose = RobotContainer.driveSubsystem.getRobotPose().relativeTo(finalAlignPose);
      ChassisSpeeds currentSpeeds = RobotContainer.driveSubsystem.getRobotChassisSpeeds();
      double velocityMagnitude = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
      
      boolean poseValid = 
        Math.abs(errorPose.getX()) <= chosenParameters.maxDistanceX() &&
        Math.abs(errorPose.getY()) <= chosenParameters.maxDistanceY() && 
        Math.abs(errorPose.getRotation().getDegrees()) <= chosenParameters.maxThetaDifference().getDegrees();
      boolean velocityValid = 
        velocityMagnitude <= chosenParameters.maxVelocity() &&
        currentSpeeds.omegaRadiansPerSecond <= chosenParameters.maxThetaVelocity().getRadians();

      return poseValid && velocityValid;
    });

    addCommands(
      // Start timing
      startTimingCommand,
      
      // Main scoring sequence
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new ParallelDeadlineGroup(
            // Wait until doNotScore is released
            new WaitUntilCommand(() -> !Controls.doNotScore.getAsBoolean() && scoreTrigger.getAsBoolean()),
            new PrepareScoreCoral()
          ),

          // Lock in current side selection
          new InstantCommand(() -> RobotContainer.dynamicPathingSubsystem.lockCoralScoringSide(true)),

          // Place on post
          new ApplySuperstructureState(chosenParameters.executeScoreState())
        ),

        pathingSubsystem.wrapPathingCommand(
          new AlignToPose(finalAlignPose)
        )
      ),

      // Rip off coral
      new ParallelDeadlineGroup(
        // Constrained backoff
        new DriveTeleop(
          this::constrainedBackoffX,
          this::constrainedBackoffY, 
          () -> Rotation2d.kZero
        ).onlyWhile(() -> DynamicPathing.getDistanceToReef() < SCORING_FINISHED_DISTANCE),
        new CoralOutake()
      ),
      
      // End timing
      endTimingCommand
    );
  }

  private double constrainedBackoffX() {
    double influence = WafflesUtilities.InvLerp(DynamicPathing.REEF_CORAL_CLEAR_DISTANCE, SCORING_FINISHED_DISTANCE, DynamicPathing.getDistanceToReef());
    influence = MathUtil.clamp(influence, 0, 1);

    System.out.println(DynamicPathing.getDistanceToReef());

    var inputVector = new Translation2d(Controls.getDriveY() , Controls.getDriveX());
    var travelDirection = new Translation2d(1, RobotContainer.dynamicPathingSubsystem.getClosestFaceAngle().plus(Rotation2d.k180deg));
    // double scaledInput = Math.max(0, WafflesUtilities.translationDotProduct(travelDirection, inputVector));
    double scaledInput = MathUtil.clamp(inputVector.getNorm(), 0, 1);

    double output = travelDirection.times(scaledInput).getX();

    return WafflesUtilities.Lerp(output, Controls.getDriveY(), influence);
  }

  private double constrainedBackoffY() {
    double influence = WafflesUtilities.InvLerp(DynamicPathing.REEF_CORAL_CLEAR_DISTANCE, SCORING_FINISHED_DISTANCE, DynamicPathing.getDistanceToReef());
    influence = MathUtil.clamp(influence, 0, 1);

    var inputVector = new Translation2d(Controls.getDriveY() , Controls.getDriveX());
    var travelDirection = new Translation2d(1, RobotContainer.dynamicPathingSubsystem.getClosestFaceAngle().plus(Rotation2d.k180deg));
    // double scaledInput = Math.max(0, WafflesUtilities.translationDotProduct(travelDirection, inputVector));
    double scaledInput = MathUtil.clamp(inputVector.getNorm(), 0, 1);

    double output = travelDirection.times(scaledInput).getY();

    return WafflesUtilities.Lerp(output, Controls.getDriveX(), influence);
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
    return new ScoreCoral(driveCommand, finalAlignPose, maxSpeed).finallyDo(() ->{
      RobotContainer.dynamicPathingSubsystem.lockCoralScoringSide(false);
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
  public static Command scoreCoralWithSettings(SuperstructureState level, boolean rightSide) {
    if (!RobotContainer.intakeSubsystem.isCoralLoaded()) {
      return new InstantCommand();
    }
    RobotContainer.dynamicPathingSubsystem.setCoralScoringLevel(level);
    RobotContainer.dynamicPathingSubsystem.setCoralScoringSide(rightSide);
    Pose2d targetCoralPose = RobotContainer.dynamicPathingSubsystem.getNearestCoralScoringLocation();

    return scoreCoralWithPath(new InstantCommand(), targetCoralPose, DynamicPathing.CORAL_PATH_END_SPEED);
  }
}
