// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.ScoringConstants.ScoringLevel;
import frc.robot.data.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.subsystems.Lights;

public class DefaultLightCommand extends Command {
  private int currentLevel = 0;

  /** Creates a new DefaultLightCommand. */
  public DefaultLightCommand() {
    addRequirements(RobotContainer.lightsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Clear any existing light patterns
    RobotContainer.lightsSubsystem.clearAllLEDs();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update lights based on robot state
    RobotContainer.lightsSubsystem.clearAllLEDs();
    RobotContainer.lightsSubsystem.updateLights();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.lightsSubsystem.clearAllLEDs();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
} 