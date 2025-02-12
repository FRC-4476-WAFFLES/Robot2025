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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.isOperatorOverride) {
      handleOverrideMode();
    } else {
      handleNormalMode();
    }
  }

  private void handleNormalMode() {
    // Get current scoring level
    ScoringLevel level = RobotContainer.dynamicPathingSubsystem.getCoralScoringLevel();
    int newLevel = 0;
    
    switch(level) {
      case L1:
        newLevel = 1;
        break;
      case L2:
        newLevel = 2;
        break;
      case L3:
        newLevel = 3;
        break;
      case L4:
        newLevel = 4;
        break;
      default:
        newLevel = 0;
        break;
    }

    // Only update if level changed
    if (newLevel != currentLevel) {
      currentLevel = newLevel;
      updateLEDsForLevel(currentLevel);
    }
  }

  private void handleOverrideMode() {
    // Get current target position
    ElevatorLevel targetPosition = RobotContainer.elevatorSubsystem.getTargetPosition();
    
    // Determine level based on target position
    int level = getLevelFromElevatorPosition(targetPosition);
    if (level != currentLevel) {
      currentLevel = level;
      updateLEDsForLevel(level);
    }
  }

  private int getLevelFromElevatorPosition(ElevatorLevel position) {
    switch(position) {
      case L1:
        return 1;
      case L2:
        return 2;
      case L3:
        return 3;
      case L4:
        return 4;
      case PROCESSOR:
        return 1; // Use level 1 lights for processor position
      default:
        return 0;
    }
  }

  private void updateLEDsForLevel(int level) {
    // Clear all LEDs first
    for (Lights.LedRange range : Lights.LedRange.values()) {
      RobotContainer.lightsSubsystem.setLEDRange(range.getStart(), range.getEnd(), Lights.LightColours.BLACK);
    }

    // Set new ranges based on level
    switch(level) {
      case 1:
        RobotContainer.lightsSubsystem.setLEDRange(
          Lights.LedRange.L1.getStart(), Lights.LedRange.L1.getEnd(), Lights.LightColours.WHITE);
        RobotContainer.lightsSubsystem.setLEDRange(
          Lights.LedRange.R1.getStart(), Lights.LedRange.R1.getEnd(), Lights.LightColours.WHITE);
        break;
      case 2:
        RobotContainer.lightsSubsystem.setLEDRange(
          Lights.LedRange.L2.getStart(), Lights.LedRange.L2.getEnd(), Lights.LightColours.WHITE);
        RobotContainer.lightsSubsystem.setLEDRange(
          Lights.LedRange.R2.getStart(), Lights.LedRange.R2.getEnd(), Lights.LightColours.WHITE);
        break;
      case 3:
        RobotContainer.lightsSubsystem.setLEDRange(
          Lights.LedRange.L3.getStart(), Lights.LedRange.L3.getEnd(), Lights.LightColours.WHITE);
        RobotContainer.lightsSubsystem.setLEDRange(
          Lights.LedRange.R3.getStart(), Lights.LedRange.R3.getEnd(), Lights.LightColours.WHITE);
        break;
      case 4:
        RobotContainer.lightsSubsystem.setLEDRange(
          Lights.LedRange.L4.getStart(), Lights.LedRange.L4.getEnd(), Lights.LightColours.WHITE);
        RobotContainer.lightsSubsystem.setLEDRange(
          Lights.LedRange.R4.getStart(), Lights.LedRange.R4.getEnd(), Lights.LightColours.WHITE);
        break;
      default:
        // All LEDs are already cleared
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Clear all LEDs when command ends
    for (Lights.LedRange range : Lights.LedRange.values()) {
      RobotContainer.lightsSubsystem.setLEDRange(range.getStart(), range.getEnd(), Lights.LightColours.BLACK);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;  // Run continuously
  }
} 