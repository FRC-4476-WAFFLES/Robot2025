// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.data.Constants;

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

public class AlgaeManipulator extends SubsystemBase {
  /** Creates a new CoralIntake. */
  private TalonFX algaeIntake;
  private TalonFX algaePivot;
  private final CurrentLimitsConfigs algaeIntakeCurrentLimit= new CurrentLimitsConfigs();
  private final CurrentLimitsConfigs algaePivotCurrentLimit= new CurrentLimitsConfigs();
  private double algaeIntakeSpeed = 0;
  private double algaePivotSpeed = 0;

  public AlgaeManipulator() {
    // talonFX configs
    algaeIntake=new TalonFX(Constants.algaeIntakeMotor);
    algaePivot=new TalonFX(Constants.algaePivotMotor);
    TalonFXConfiguration algaeIntakeConfigs = new TalonFXConfiguration();
    TalonFXConfiguration algaePivotConfigs = new TalonFXConfiguration();
    algaeIntakeCurrentLimit.StatorCurrentLimit=60;
    algaePivotCurrentLimit.StatorCurrentLimit=60;
    algaeIntakeCurrentLimit.StatorCurrentLimitEnable=true;
    algaePivotCurrentLimit.StatorCurrentLimitEnable=true;
    algaeIntakeConfigs.CurrentLimits=algaeIntakeCurrentLimit;
    algaePivotConfigs.CurrentLimits=algaePivotCurrentLimit;
    algaeIntake.getConfigurator().apply(algaeIntakeConfigs);
    algaePivot.getConfigurator().apply(algaePivotConfigs);
    // motion magic setup (from Elevator subsystem) -- not used yet in algae manipulator code, ADD IF NECESSARY
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = 110; // Using the existing velocity value
    motionMagicConfigs.MotionMagicAcceleration = 190; // Using the existing acceleration value
    motionMagicConfigs.MotionMagicJerk = 1900; // Setting jerk to 10x acceleration as a starting point
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final DutyCycleOut algaeIntakeDutyCycle = new DutyCycleOut(0);
    algaeIntake.setControl(algaeIntakeDutyCycle.withOutput(algaeIntakeSpeed));

    final DutyCycleOut algaePivotDutyCycle = new DutyCycleOut(0);
    algaePivot.setControl(algaePivotDutyCycle.withOutput(algaePivotSpeed));
  }

  public void setAlgaeIntakeSpeed(double algaeIntakeSpeed){
    this.algaeIntakeSpeed=algaeIntakeSpeed;
  }
  public void setAlgaePivotSpeed(double algaePivotSpeed){
    this.algaePivotSpeed=algaePivotSpeed;
  }
}
