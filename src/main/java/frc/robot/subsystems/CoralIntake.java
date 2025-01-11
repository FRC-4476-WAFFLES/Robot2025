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

public class CoralIntake extends SubsystemBase {
  /** Creates a new CoralIntake. */
  private TalonFX coralIntake;
  private TalonFX coralPivot;
  private final CurrentLimitsConfigs coralIntakeCurrentLimit= new CurrentLimitsConfigs();
  private final CurrentLimitsConfigs coralPivotCurrentLimit= new CurrentLimitsConfigs();
  private double coralIntakeSpeed = 0;
  private double coralPivotSpeed = 0;

  public CoralIntake() {
    coralIntake=new TalonFX(Constants.intakeMotor);
    coralPivot=new TalonFX(Constants.pivotMotor);
    TalonFXConfiguration coralIntakeConfigs = new TalonFXConfiguration();
    TalonFXConfiguration coralPivotConfigs = new TalonFXConfiguration();
    coralIntakeCurrentLimit.StatorCurrentLimit=60;
    coralPivotCurrentLimit.StatorCurrentLimit=60;
    coralIntakeCurrentLimit.StatorCurrentLimitEnable=true;
    coralPivotCurrentLimit.StatorCurrentLimitEnable=true;
    coralIntakeConfigs.CurrentLimits=coralIntakeCurrentLimit;
    coralPivotConfigs.CurrentLimits=coralPivotCurrentLimit;
    coralIntake.getConfigurator().apply(coralIntakeConfigs);
    coralPivot.getConfigurator().apply(coralPivotConfigs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final DutyCycleOut coralIntakeDutyCycle = new DutyCycleOut(0);
    coralIntake.setControl(coralIntakeDutyCycle.withOutput(coralIntakeSpeed));

    final DutyCycleOut coralPivotDutyCycle = new DutyCycleOut(0);
    coralPivot.setControl(coralPivotDutyCycle.withOutput(coralPivotSpeed));
  }

  public void setCoralIntakeSpeed(double coralIntakeSpeed){
    this.coralIntakeSpeed=coralIntakeSpeed;
  }
  public void setCoralPivotSpeed(double coralPivotSpeed){
    this.coralPivotSpeed=coralPivotSpeed;
  }
}
