// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.test;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class CoralIntake extends SubsystemBase {
  /** Creates a new CoralIntake. */
  private TalonFX coralIntake;
  private final CurrentLimitsConfigs coralIntakeCurrentLimit= new CurrentLimitsConfigs();
  private double speed = 0;
  public CoralIntake() {
    coralIntake=new TalonFX(Constants.intakeMotor);
    TalonFXConfiguration coralConfigs = new TalonFXConfiguration();
    coralIntakeCurrentLimit.StatorCurrentLimit=60;
    coralIntakeCurrentLimit.StatorCurrentLimitEnable=true;
    coralConfigs.CurrentLimits=coralIntakeCurrentLimit;
    coralIntake.getConfigurator().apply(coralConfigs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final DutyCycleOut coralIntakeDutyCycle = new DutyCycleOut(0);
    coralIntake.setControl(coralIntakeDutyCycle.withOutput(speed));
  }

  public void setCoralIntakeSpeed(double speed){
    this.speed=speed;
  }
}
