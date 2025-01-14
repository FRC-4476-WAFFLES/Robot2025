// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.data.Constants;

import static frc.robot.RobotContainer.funnel;

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

public class Funnel extends SubsystemBase {
  /** Creates a new CoralIntake. */
  private TalonFX funnel;
  private final CurrentLimitsConfigs funnelCurrentLimit= new CurrentLimitsConfigs();
  private double funnelSpeed = 0;

  public Funnel() {
    // talonFX configs
    funnel=new TalonFX(Constants.funnelMotor);
    TalonFXConfiguration funnelConfigs = new TalonFXConfiguration();
    funnelCurrentLimit.StatorCurrentLimit=60;
    funnelCurrentLimit.StatorCurrentLimitEnable=true;
    funnelConfigs.CurrentLimits=funnelCurrentLimit;
    funnel.getConfigurator().apply(funnelConfigs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final DutyCycleOut funnelDutyCycle = new DutyCycleOut(0);
    funnel.setControl(funnelDutyCycle.withOutput(funnelSpeed));
  }
  public void setFunnelSpeed(double funnelSpeed){
    this.funnelSpeed=funnelSpeed;
  }
  
}
