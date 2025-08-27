// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Controls;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.superstructure.ApplySuperstructureState;
import frc.robot.data.Constants.ScoringConstants;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;

/** Factory for algae toss command */
public class ScoreNet {

    public static Command getScoreNetCommand(double targetNetX, Supplier<Rotation2d> targetNetRotation, boolean doAlign) {
        Command alignCommand = new DriveTeleop(
            () -> targetNetX, true, // Just pid to setpoint should be () -> targetNetX
            Controls::getDriveX, false,
            targetNetRotation, true
        );

        if (!doAlign) {
            alignCommand = new InstantCommand();
        }

        return Commands.deadline(
            // Score sequence (Operator controlled)
            Commands.sequence(
                // Run intake in during NET_PREP position
                Commands.parallel(
                    new ApplySuperstructureState(SuperstructureState.NET_PREP)
                ),
                Commands.runOnce(() -> RobotContainer.superstructure.pivot.setIsThrowingAlgae(true)),
                Commands.waitSeconds(0.2),
                algaeToss()
            ),
            // Alignment
            alignCommand
        ).finallyDo(() -> {
            RobotContainer.superstructure.applySuperstructureState(SuperstructureState.ZERO);

            RobotContainer.superstructure.pivot.setIsThrowingAlgae(false);
            RobotContainer.intakeSubsystem.setIntakeSpeed(0); // Ensure intake is stopped
            RobotContainer.intakeSubsystem.setDutyCycle(0);
        });
    }

    /**
     * The algae toss sequence
     * @return A command
     */
    private static Command algaeToss() {
        return Commands.sequence(
            Commands.waitUntil(Controls.doNotScore.negate()),
            Commands.runOnce(() -> RobotContainer.intakeSubsystem.setIntakeSpeed(0)),
            Commands.parallel(
                new ApplySuperstructureState(SuperstructureState.NET),
                // Release at the same point
                Commands.sequence(
                    Commands.waitUntil(() ->   
                        RobotContainer.superstructure.pivot.getPivotPosition() <= ScoringConstants.ALGAE_TOSS_PIVOT_ANGLE
                    ),
                    Commands.runOnce(() -> {RobotContainer.intakeSubsystem.setDutyCycle(-1);}),
                    Commands.waitSeconds(0.4)
                ).finallyDo(() -> {
                    RobotContainer.intakeSubsystem.setIntakeSpeed(0);
                })
            )
        );
    }
}
