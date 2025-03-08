// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Controls;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.superstructure.ApplyScoringSetpoint;
import frc.robot.data.Constants.ScoringConstants;
import frc.robot.data.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.data.Constants.ManipulatorConstants.PivotPosition;
import frc.robot.data.Constants.ScoringConstants.ScoringLevel;

/** Factory for algae toss command */
public class ScoreNet {

    public static Command getScoreNetCommand(double targetNetX, Rotation2d targetNetRotation) {
        return Commands.parallel(
            // Alignment
            new DriveTeleop(
                Controls::getDriveY, false, // Just pid to setpoint should be () -> targetNetX
                Controls::getDriveX, false,
                () -> targetNetRotation, true
            ),
            // Score sequence (Operator controlled)
            Commands.sequence(
                new ApplyScoringSetpoint(ScoringLevel.NET_PREP),
                Commands.runOnce(() -> RobotContainer.pivotSubsystem.setIsThrowingAlgae(true)),
                algaeToss()
            )
        ).finallyDo(() -> {
            RobotContainer.elevatorSubsystem.setElevatorSetpoint(ElevatorLevel.REST_POSITION);
            RobotContainer.pivotSubsystem.setPivotSetpoint(PivotPosition.CLEARANCE_POSITION);

            RobotContainer.pivotSubsystem.setIsThrowingAlgae(false);
        });
    }

    /**
     * The algae toss sequence
     * @return A command
     */
    private static Command algaeToss() {
        return Commands.sequence(
            Commands.waitUntil(Controls.algaeOut),
            Commands.parallel(
                new ApplyScoringSetpoint(ScoringLevel.NET),
                // Release at the same point
                Commands.sequence(
                    Commands.waitUntil(() ->   
                        RobotContainer.pivotSubsystem.getPivotPosition() <= ScoringConstants.ALGAE_TOSS_PIVOT_ANGLE
                    ),
                    Commands.runOnce(() -> {RobotContainer.intakeSubsystem.setIntakeSpeed(-300);}),
                    Commands.waitSeconds(0.5)
                ).finallyDo(() -> {
                    RobotContainer.intakeSubsystem.setIntakeSpeed(0);
                })
            )
        );
    }
}
