// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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

public class AlgaeManipulator extends SubsystemBase {
  /** Creates a new CoralIntake. */
  private TalonFX algaeIntake;
  private TalonFX algaePivot;
  private final DutyCycleEncoder pivotAbsoluteEncoder;
  private final CurrentLimitsConfigs algaeIntakeCurrentLimit= new CurrentLimitsConfigs();
  private final CurrentLimitsConfigs algaePivotCurrentLimit= new CurrentLimitsConfigs();
  private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private double algaeIntakeSpeed = 0;
  private double algaePivotSpeed = 0;
  private double pivotTargetPositionRotations=0;
  
  public enum pivotPositions {
    insideTheRobot(0),
    outsideTheRobot(90);

    private double pivotDegrees;

    pivotPositions(double pivotDegrees) {
      this.pivotDegrees = pivotDegrees;
    }

    public double getDegrees() {
      return pivotDegrees;
    }
  }
  pivotPositions currentPivotDegree=pivotPositions.insideTheRobot;
  public AlgaeManipulator() {
    // talonFX configs
    algaeIntake=new TalonFX(Constants.algaeIntakeMotor);
    algaePivot=new TalonFX(Constants.algaePivotMotor);
    pivotAbsoluteEncoder= new DutyCycleEncoder(Constants.pivotAbsoluteEncoder);
    TalonFXConfiguration algaeIntakeConfigs = new TalonFXConfiguration();
    TalonFXConfiguration algaePivotConfigs = new TalonFXConfiguration();
    algaeIntakeCurrentLimit.StatorCurrentLimit=60;
    algaePivotCurrentLimit.StatorCurrentLimit=60;
    algaeIntakeCurrentLimit.StatorCurrentLimitEnable=true;
    algaePivotCurrentLimit.StatorCurrentLimitEnable=true;
    algaeIntakeConfigs.CurrentLimits=algaeIntakeCurrentLimit;
    algaePivotConfigs.CurrentLimits=algaePivotCurrentLimit;
    
    // motion magic setup (from Elevator subsystem) -- not used yet in algae manipulator code, ADD IF NECESSARY
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = 110; // Using the existing velocity value
    motionMagicConfigs.MotionMagicAcceleration = 190; // Using the existing acceleration value
    motionMagicConfigs.MotionMagicJerk = 1900; // Setting jerk to 10x acceleration as a starting point
    algaePivotConfigs.MotionMagic = motionMagicConfigs;

    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0; // Keeping the existing value
    slot0Configs.kP = 2; // Keeping the existing value
    slot0Configs.kI = 0; // Keeping the existing value
    slot0Configs.kD = 0.01; // Keeping the existing value
    algaePivotConfigs.Slot0 = slot0Configs;

    algaeIntake.getConfigurator().apply(algaeIntakeConfigs);
    algaePivot.getConfigurator().apply(algaePivotConfigs);

    algaePivot.setPosition(pivotAbsoluteEncoder.get()+Constants.pivotAbsoluteEncoderOffset);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final DutyCycleOut algaeIntakeDutyCycle = new DutyCycleOut(0);
    algaeIntake.setControl(algaeIntakeDutyCycle.withOutput(algaeIntakeSpeed));
    pivotTargetPositionRotations=currentPivotDegree.getDegrees();
    executeClimberMotionMagic();
  }

  private void executeClimberMotionMagic() {
    motionMagicRequest.Position = pivotTargetPositionRotations;
    motionMagicRequest.Slot = 0; // Use the Slot0 gains
    algaePivot.setControl(motionMagicRequest);
  }

  public void setAlgaeIntakeSpeed(double algaeIntakeSpeed){
    this.algaeIntakeSpeed=algaeIntakeSpeed;
  }
  public void setAlgaePivotSpeed(double algaePivotSpeed){
    this.algaePivotSpeed=algaePivotSpeed;
  }
  public boolean isAlgaePivotCorrectPos(){
    return algaePivot.getPosition().getValueAsDouble()==pivotTargetPositionRotations;
  }

    /**
   * Detects if a algae is present in the intake based on current draw.
   * @return true if a algae is detected (current exceeds threshold), false otherwise.
   */
  public boolean isAlgaeCurrentDetection() {
    return algaePivot.getStatorCurrent().getValueAsDouble() > 34;
  }

  public void setInsideTheRobot() {
    currentPivotDegree = pivotPositions.insideTheRobot;
  }

  public void setOutsideTheRobot() {
    currentPivotDegree =pivotPositions.outsideTheRobot;
  }

}
