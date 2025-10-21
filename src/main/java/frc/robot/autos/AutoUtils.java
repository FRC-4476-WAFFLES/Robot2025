// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.AutoAlignToPose;
import frc.robot.data.AutoCoordinates;
import frc.robot.subsystems.DynamicPathing;
import frc.robot.utils.WafflesUtilities;
import frc.robot.RobotContainer;
public class AutoUtils {
    public static final double maxApproachOffsetDistance = 1.5;
    public static Pose2d evaluateCoralApproachGoal(Pose2d target) {
        Pose2d offset = WafflesUtilities.FlipIfRedAlliance(RobotContainer.driveSubsystem.getRobotPose()).relativeTo(target);
        
        return target.transformBy(new Transform2d(
            -maxApproachOffsetDistance * (Math.abs(offset.getY()) / 3),
            0,
            Rotation2d.kZero
        ));
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
                NamedCommands.getCommand("Set Position L2") // Legacy name
            ),

            Commands.either(
                NamedCommands.getCommand("Autoscore L4 Left"), 
                NamedCommands.getCommand("Autoscore L4 Right"), 
                () -> left
            )
        );
    }

    public static Command driveAwayFromPost(Pose2d post, Pose2d target) {
        return new AutoAlignToPose(() -> evaluateCoralBackoffGoal(post, target));
    }

    public static Command resetOdometry(Pose2d instantPose){
        return new InstantCommand(
            ()->{
                RobotContainer.driveSubsystem.resetTranslation(
                WafflesUtilities.FlipIfRedAlliance(instantPose).getTranslation()
                );
                RobotContainer.driveSubsystem.resetRotation(
                WafflesUtilities.FlipIfRedAlliance(instantPose).getRotation()
                );
            });

         
    }
}
