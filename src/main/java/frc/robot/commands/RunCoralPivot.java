// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.RobotContainer.coralPivot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.data.Constants;
import frc.robot.subsystems.CoralManipulator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunCoralPivot extends Command {
  /** Creates a new RunPivot. */
  public RunCoralPivot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralPivot);
    // motion magic setup (from Elevator subsystem) -- not used yet in this code, ADD IF NECESSARY
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = 110; // Using the existing velocity value
    motionMagicConfigs.MotionMagicAcceleration = 190; // Using the existing acceleration value
    motionMagicConfigs.MotionMagicJerk = 1900; // Setting jerk to 10x acceleration as a starting point
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    executeCoralPivotMotionMagic();
  }

  private void executeCoralPivotMotionMagic() {
    motionMagicRequest.Position = coralPivotTargetPositionRotations;
    motionMagicRequest.Slot = 0; // Use the Slot0 gains
    coralPivot.setControl(motionMagicRequest);

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralPivot.setCoralPivotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
