// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.data.Constants.ManipulatorConstants.PivotPosition;

public class DefaultPosition extends Command {
  /** Creates a new DefaultPosition. */
  public DefaultPosition() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.elevatorSubsystem, RobotContainer.manipulatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.elevatorSubsystem.setElevatorSetpoint(ElevatorLevel.REST_POSITION);
    RobotContainer.manipulatorSubsystem.setPivotSetpoint(PivotPosition.ZERO);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Run continuously until interrupted
  }
} 