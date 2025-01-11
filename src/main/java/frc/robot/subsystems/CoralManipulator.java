// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

public class CoralManipulator extends SubsystemBase {
  /** Creates a new CoralIntake. */
  private TalonFX coralGrab;
  private TalonFX coralPivot;
  private final CurrentLimitsConfigs coralGrabCurrentLimit= new CurrentLimitsConfigs();
  private final CurrentLimitsConfigs coralPivotCurrentLimit= new CurrentLimitsConfigs();
  private double coralGrabSpeed = 0;
  private double coralPivotSpeed = 0;

  public CoralManipulator() {
    // talonFX configs
    coralGrab=new TalonFX(Constants.coralGrabMotor);
    coralPivot=new TalonFX(Constants.coralPivotMotor);
    TalonFXConfiguration coralGrabConfigs = new TalonFXConfiguration();
    TalonFXConfiguration coralPivotConfigs = new TalonFXConfiguration();
    coralGrabCurrentLimit.StatorCurrentLimit=60;
    coralPivotCurrentLimit.StatorCurrentLimit=60;
    coralGrabCurrentLimit.StatorCurrentLimitEnable=true;
    coralPivotCurrentLimit.StatorCurrentLimitEnable=true;
    coralGrabConfigs.CurrentLimits=coralGrabCurrentLimit;
    coralPivotConfigs.CurrentLimits=coralPivotCurrentLimit;
    coralGrab.getConfigurator().apply(coralGrabConfigs);
    coralPivot.getConfigurator().apply(coralPivotConfigs);
    // motion magic setup (from Elevator subsystem) -- not used yet in coral manipulator code, ADD IF NECESSARY
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = 110; // Using the existing velocity value
    motionMagicConfigs.MotionMagicAcceleration = 190; // Using the existing acceleration value
    motionMagicConfigs.MotionMagicJerk = 1900; // Setting jerk to 10x acceleration as a starting point
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final DutyCycleOut coralGrabDutyCycle = new DutyCycleOut(0);
    coralGrab.setControl(coralGrabDutyCycle.withOutput(coralGrabSpeed));

    final DutyCycleOut coralPivotDutyCycle = new DutyCycleOut(0);
    coralPivot.setControl(coralPivotDutyCycle.withOutput(coralPivotSpeed));
  }

  public void setCoralGrabSpeed(double coralGrabSpeed){
    this.coralGrabSpeed=coralGrabSpeed;
  }
  public void setCoralPivotSpeed(double coralPivotSpeed){
    this.coralPivotSpeed=coralPivotSpeed;
  }
}
