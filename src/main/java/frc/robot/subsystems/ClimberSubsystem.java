// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

public class ClimberSubsystem extends SubsystemBase {
  
  // one leader motor, one follower motor to align robot with cage
  // motor to lower arm, which holds the cage and lifts the robot
  // intake / outtake motor

  // TODO
  // in/out motor

  public final TalonFX climberMotorLeader;
  public final TalonFX climberMotorFollower;
  public final TalonFX alignmentMotor;

  private final CurrentLimitsConfigs climberCurrentLimits = new CurrentLimitsConfigs();
  private final CurrentLimitsConfigs alignmentCurrentLimit= new CurrentLimitsConfigs();

  private double rotationPosition;
  
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem()
  {
    climberMotorLeader = new TalonFX(9);
    climberMotorFollower = new TalonFX(10);
    alignmentMotor = new TalonFX(16);
    // climber motor follower is inverted
    climberMotorFollower.setControl(new Follower(Constants.climberLeader, true));

    // create a configuration object for the climber motor
    TalonFXConfiguration climberConfig = new TalonFXConfiguration();
    TalonFXConfiguration alignmentConfig = new TalonFXConfiguration();

    // current limits
    climberCurrentLimits.StatorCurrentLimit = 40;
    climberCurrentLimits.StatorCurrentLimitEnable = true;
    alignmentCurrentLimit.StatorCurrentLimit=60;
    alignmentCurrentLimit.StatorCurrentLimitEnable=true;

    // apply the configuration to the climber motor
    climberConfig.CurrentLimits = climberCurrentLimits;
    alignmentConfig.CurrentLimits = alignmentCurrentLimit;

    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0; // Keeping the existing value
    slot0Configs.kP = 2; // Keeping the existing value
    slot0Configs.kI = 0; // Keeping the existing value
    slot0Configs.kD = 0.01; // Keeping the existing value

    climberConfig.Slot0 = slot0Configs;

    // Configure MotionMagic
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = 110; // Using the existing velocity value
    motionMagicConfigs.MotionMagicAcceleration = 190; // Using the existing acceleration value
    motionMagicConfigs.MotionMagicJerk = 1900; // Setting jerk to 10x acceleration as a starting point
    climberConfig.MotionMagic = motionMagicConfigs;

    climberMotorLeader.getConfigurator().apply(climberConfig);
    alignmentMotor.getConfigurator().apply(alignmentConfig);
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    
  }

  public void SetRotationPosition(double rotPos)
  {
    this.rotationPosition = rotPos;
  }
}
