// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.AlignToCoral;
import frc.robot.commands.drive.AutoAlignToPose;
import frc.robot.commands.intake.CoralOutake;
import frc.robot.commands.superstructure.ExecuteHandoff;
import frc.robot.commands.superstructure.SuperstructureControl;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.subsystems.DynamicPathing;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.utils.WafflesUtilities;

public class AutoUtils {
    public static final double maxApproachOffsetDistance = 1.5;
    public static Pose2d evaluateCoralApproachGoal(Pose2d target) {
        Pose2d offset = WafflesUtilities.FlipIfRedAlliance(RobotContainer.driveSubsystem.getRobotPose()).relativeTo(target);
        double goalBackshift = Math.abs(offset.getY()) / 3;

        if (goingToHitReef()) {
            goalBackshift = Math.max(goalBackshift, 0.13);
        }

        return target.transformBy(new Transform2d(
            -maxApproachOffsetDistance * goalBackshift,
            0,
            Rotation2d.kZero
        ));
    }

    private static boolean goingToHitReef() {
        return !RobotContainer.intakeSubsystem.isCoralLoaded() || 
        (!RobotContainer.superstructure.pivot.pastReefHitAngle() && RobotContainer.superstructure.elevator.getElevatorPositionMeters() < 0.9);
    }

    public static Pose2d evaluateCoralBackoffGoal(Pose2d postPose, Pose2d target) {
        Pose2d offset = WafflesUtilities.FlipIfRedAlliance(RobotContainer.driveSubsystem.getRobotPose()).relativeTo(postPose);

        if (offset.getTranslation().getNorm() > 0.3) {
            return target;
        }
        double lerpValue = MathUtil.clamp( 
            WafflesUtilities.InvLerp(0.1, 0.6, offset.getTranslation().getNorm()),
        0, 1);

        return postPose.transformBy(
            new Transform2d(
                -0.7,
                0,
                Rotation2d.kZero
            )
        ).interpolate(target, lerpValue);
    }

    public static Command prepareAndScore(Pose2d post, boolean left) {
        return Commands.sequence( 
            Commands.parallel(
                new AutoAlignToPose(() -> evaluateCoralApproachGoal(post)),
                Commands.sequence(
                    new ExecuteHandoff().onlyIf(() -> RobotContainer.triggerHandoff.getAsBoolean()), // Handoff coral while driving to score
                    SuperstructureControl.L4ScorePrepCommand()
                )
            ),

            Commands.either(
                NamedCommands.getCommand("Autoscore L4 Left"), 
                NamedCommands.getCommand("Autoscore L4 Right"), 
                () -> left
            )
        );
    }

    private static boolean shouldSwitchToHunting() {
        return DynamicPathing.getDistanceToReef() > 2 + DynamicPathing.REEF_INRADIUS && RobotContainer.telemetry.coralTracking.hasTarget();
    }

    private static boolean coralTargetLost() {
        return !RobotContainer.telemetry.coralTracking.hasTarget();
    }

    public static Command huntCoral() {
        return new AlignToCoral()
        .onlyWhile(() -> !RobotContainer.groundSuperstructure.isHandoffHappening());
    }

    public static Command driveAwayFromPost(Pose2d post, Pose2d target) {
        return new AutoAlignToPose(() -> evaluateCoralBackoffGoal(post, target));
    }

    public static Command resetOdometry(Pose2d instantPose){
        if (!CodeConstants.RESET_ODOMETRY_AUTO_START) {
            return new InstantCommand();
        }
        return new InstantCommand(() -> {
            RobotContainer.driveSubsystem.resetTranslation(
                WafflesUtilities.FlipIfRedAlliance(instantPose).getTranslation()
            );
            RobotContainer.driveSubsystem.resetRotation(
                WafflesUtilities.FlipIfRedAlliance(instantPose).getRotation()
            );
        });
    }

         
    private static Command placeAndAwaitIntake() {
        return Commands.sequence(
            new CoralOutake(),
            Commands.runOnce(() -> RobotContainer.groundSuperstructure.startHandoffIntake()),
            Commands.waitUntil(() -> DynamicPathing.isElevatorRetractionSafe()),      
            Commands.runOnce(() -> RobotContainer.superstructure.applySuperstructureState(SuperstructureState.HANDOFF_READY)),
            Commands.waitUntil(() -> RobotContainer.triggerHandoff.getAsBoolean())
            // Handoff gets executed when score command starts
        );
    }

    public static Command intakeSequence(Pose2d post, Pose2d target) {
        return Commands.deadline(
            placeAndAwaitIntake(),
            Commands.sequence( 
                driveAwayFromPost(post, target).until(AutoUtils::shouldSwitchToHunting),
                huntCoral().until(AutoUtils::coralTargetLost)
            ).repeatedly()
        );
    }

    public static Command lolipopIntakeSequence(Pose2d post, Pose2d target) {
        return Commands.deadline(
            placeAndAwaitIntake(),
            Commands.sequence( 
                driveAwayFromPost(post, target)
            )
        );
    }
}
