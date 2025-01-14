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

public class CoralIntake extends SubsystemBase {
  /** Creates a new CoralIntake. */
  private TalonFX coralIntake;
  private final CurrentLimitsConfigs coralIntakeCurrentLimit= new CurrentLimitsConfigs();
  private double coralIntakeSpeed = 0;

  public CoralIntake() {
    // talonFX configs
    coralIntake=new TalonFX(Constants.coralIntakeMotor);
    TalonFXConfiguration coralIntakeConfigs = new TalonFXConfiguration();
    coralIntakeCurrentLimit.StatorCurrentLimit=60;
    coralIntakeCurrentLimit.StatorCurrentLimitEnable=true;
    coralIntakeConfigs.CurrentLimits=coralIntakeCurrentLimit;
    coralIntake.getConfigurator().apply(coralIntakeConfigs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final DutyCycleOut coralIntakeDutyCycle = new DutyCycleOut(0);
    coralIntake.setControl(coralIntakeDutyCycle.withOutput(coralIntakeSpeed));
  }
  public void setCoralIntakeSpeed(double coralIntakeSpeed){
    this.coralIntakeSpeed=coralIntakeSpeed;
  }
  
}
