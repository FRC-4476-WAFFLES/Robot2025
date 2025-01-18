// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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

public class CoralManipulator extends SubsystemBase {
  /** Creates a new CoralIntake. */
  private final DutyCycleEncoder coralManipulatorAbsoluteEncoder;
  private TalonFX coralIntake;
  private TalonFX coralPivot;
  private final CurrentLimitsConfigs coralIntakeCurrentLimit= new CurrentLimitsConfigs();
  private final CurrentLimitsConfigs coralPivotCurrentLimit= new CurrentLimitsConfigs();
  private double coralIntakeSpeed = 0;
  private double coralPivotSpeed = 0;
  public enum manipulatorPosition {
   L3(180),
   L2(120),
   L1(60),
   L0(0);
   private double position;
  
   manipulatorPosition(double position) {
     this.position = position;
   }

   public double getPosition() {
     return position;
   }
  }
  public CoralManipulator() {
    // talonFX configs
    coralIntake=new TalonFX(Constants.coralIntakeMotor);
    coralPivot=new TalonFX(Constants.coralPivotMotor);
    coralManipulatorAbsoluteEncoder = new DutyCycleEncoder(Constants.coralManipulatorAbsoluteEncoder);
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
    // motion magic setup (from Elevator subsystem) -- not used yet in coral manipulator code, ADD IF NECESSARY
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = 110; // Using the existing velocity value
    motionMagicConfigs.MotionMagicAcceleration = 190; // Using the existing acceleration value
    motionMagicConfigs.MotionMagicJerk = 1900; // Setting jerk to 10x acceleration as a starting point
    coralPivot.setPosition(coralManipulatorAbsoluteEncoder.get()+Constants.coralManipulatorAbsoluteEncoderOffset);
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
  public boolean isCoralCurrentDetection() {
    return coralIntake.getStatorCurrent().getValueAsDouble() > 34;//change the current threshold
  }
}
