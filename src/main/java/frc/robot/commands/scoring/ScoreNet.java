// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Controls;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.intake.AlgaeOutake;
import frc.robot.commands.superstructure.ApplySuperstructureState;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;

/** Factory for algae placement command */
public class ScoreNet {

    public static Command getScoreNetCommand(double targetNetX, Supplier<Rotation2d> targetNetRotation, boolean doAlign, boolean isFrontScoring) {
        Command alignCommand = new DriveTeleop(
            () -> targetNetX, true, // Just pid to setpoint should be () -> targetNetX
            Controls::getDriveX, false,
            targetNetRotation, true
        );

        if (!doAlign) {
            alignCommand = new InstantCommand();
        }

        return Commands.deadline(
            Commands.sequence(
                new ApplySuperstructureState(SuperstructureState.NET_PREP),
                new ApplySuperstructureState(isFrontScoring ? SuperstructureState.NET_FRONT : SuperstructureState.NET_BACK),
                // Commands.waitUntil(Controls.doNotScore.negate()),
                new AlgaeOutake()
            ),
            alignCommand
        ).finallyDo(() -> {
            RobotContainer.superstructure.applySuperstructureState(SuperstructureState.ZERO);
            RobotContainer.intakeSubsystem.setIntakeSpeed(0); 
            RobotContainer.intakeSubsystem.setDutyCycle(0);

            // In case we're scoring front don't hit the crossbar
            if (isFrontScoring) {
                CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                        new ApplySuperstructureState(SuperstructureState.NET_FRONT_CLEAR),
                        new ApplySuperstructureState(SuperstructureState.NET_FRONT_CLEAR_FINISHED)
                    )
                );
            }
        });
    }

}
