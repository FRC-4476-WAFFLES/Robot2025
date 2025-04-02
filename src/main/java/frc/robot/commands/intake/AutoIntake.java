// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.util.Arrays;
import java.util.HashSet;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.commands.AlignToPose;
import frc.robot.commands.superstructure.ApplyScoringSetpoint;
import frc.robot.data.Constants.FieldConstants;
import frc.robot.data.Constants.ScoringConstants.ScoringLevel;
import frc.robot.subsystems.DynamicPathing;
import frc.robot.utils.WafflesUtilities;

/** Auto Intake */
public class AutoIntake {
    public static final HashSet<Subsystem> commandRequirements = new HashSet<>(Arrays.asList(
        RobotContainer.driveSubsystem, 
        RobotContainer.pivotSubsystem, 
        RobotContainer.elevatorSubsystem, 
        RobotContainer.intakeSubsystem
    ));

    public static Command GetAutoIntakeCommand() {
        return Commands.defer(
            () -> {
                Pose2d currentPose = WafflesUtilities.FlipIfRedAlliance(RobotContainer.driveSubsystem.getRobotPose());

                Translation2d targetTranslation;
                if (currentPose.getY() > (FlippingUtil.fieldSizeY / 2)) {
                    targetTranslation = FieldConstants.HumanPlayerLeftPos;
                } else {
                    targetTranslation = FieldConstants.HumanPlayerRightPos;
                }
                

                Rotation2d targetRotation = RobotContainer.dynamicPathingSubsystem.getHumanPlayerPickupAngle();

                Pose2d chosenStationPose = new Pose2d(targetTranslation, targetRotation);
                chosenStationPose = WafflesUtilities.FlipIfRedAlliance(chosenStationPose);

                SmartDashboard.putNumberArray("TargetPose Align", new double[] {
                    chosenStationPose.getX(),
                    chosenStationPose.getY(),
                    chosenStationPose.getRotation().getDegrees()
                });

                return Commands.deadline(
                    new CoralIntake(),
                    new AlignToPose(chosenStationPose),
                    new ApplyScoringSetpoint(ScoringLevel.CORAL_INTAKE)
                );
            }, 
            commandRequirements
        );
    }
}
