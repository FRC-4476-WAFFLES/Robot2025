// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import java.util.Arrays;
import java.util.HashSet;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
import frc.robot.commands.intake.CoralOutake;
import frc.robot.commands.scoring.FinalAlignCoral;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.scoring.PickupAlgea;

public class ScoreCoral extends SequentialCommandGroup {
  private static final double waitBeforeScoreL4 = 0.1;
  // All scoring commands require these subsystems
  public static final HashSet<Subsystem> commandRequirements = new HashSet<>(Arrays.asList(
    RobotContainer.driveSubsystem, 
    RobotContainer.pivotSubsystem, 
    RobotContainer.elevatorSubsystem, 
    RobotContainer.intakeSubsystem
  ));

  /** Creates a new ScoreCoral. */
  private ScoreCoral(Command driveCommand, Pose2d finalAlignPose) {
    var pathingSubsystem = RobotContainer.dynamicPathingSubsystem;
    addCommands(
      new ParallelCommandGroup(
        pathingSubsystem.wrapPathingCommand(
          new SequentialCommandGroup(
            driveCommand,
            new FinalAlignCoral(finalAlignPose)
          )
        ),
        new PrepareScoreCoral()
      ),
      // Wait until doNotScore is released
      new WaitUntilCommand(() -> !Controls.doNotScore.getAsBoolean()),
      new CoralOutake()
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
        RobotContainer.pivotSubsystem.setPivotSetpoint(PivotPosition.ZERO);
      } else {
        RobotContainer.pivotSubsystem.setPivotSetpoint(PivotPosition.CLEARANCE_POSITION);
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
      new WaitUntilCommand(() -> {
        // Check if the button is held down AND other conditions are met
        boolean shouldPickupAlgae = Controls.algaeAfterScoreButton.getAsBoolean() && 
            DynamicPathing.isRobotInRangeOfReefPathing() && 
            !RobotContainer.intakeSubsystem.isCoralLoaded() &&
            !RobotContainer.intakeSubsystem.isAlgaeLoaded() &&
            Controls.dynamicPathingButton.getAsBoolean();
        
        if (shouldPickupAlgae) {
          // Use the new helper method in DynamicPathing to create the algae pickup command
          var dynamicPathing = RobotContainer.dynamicPathingSubsystem;
          Command algaeCommand = dynamicPathing.createAlgaePickupCommand();
          
          if (algaeCommand != null) {
            dynamicPathing.wrapActionStateCommand(algaeCommand).schedule();
          }
        }
        
        // Always end this command immediately after deciding whether to run the next command
        return true;
      })
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
