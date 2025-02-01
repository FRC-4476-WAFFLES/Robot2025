// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.data.Constants;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorLevel;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import au.grapplerobotics.LaserCan;


public class Manipulator extends SubsystemBase {
  /** Creates a new CoralIntake. */
  private TalonFX intake;
  private TalonFX pivot;
  private final DutyCycleEncoder pivotAbsoluteEncoder;
  private LaserCan laserCanCamera;

  private final DutyCycleOut algaeIntakeDutyCycle = new DutyCycleOut(0);
  private double intakeSpeed = 0;
  
  private double pivotTargetPositionRotations=0;
   private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private final CurrentLimitsConfigs pivotCurrentLimit= new CurrentLimitsConfigs();
  
  private final CurrentLimitsConfigs intakeCurrentLimit= new CurrentLimitsConfigs();

  public enum pivotPositions {
    ALGAE(6789),
    CORALINTAKE(12345),
    NET(10),
    L3(50.0),
    L2(27.0),
    L0(2);

    private double pivotDegrees;

    pivotPositions(double pivotDegrees) {
      this.pivotDegrees = pivotDegrees;
    }

    public double getDegrees() {
      return pivotDegrees;
    }
  }

  public Manipulator() {
    
    pivot=new TalonFX(Constants.CANIds.pivotMotor);
    pivotAbsoluteEncoder= new DutyCycleEncoder(Constants.CANIds.pivotAbsoluteEncoder);
    TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();

    pivotCurrentLimit.StatorCurrentLimit=60;
    pivotCurrentLimit.StatorCurrentLimitEnable=true;
    pivotConfigs.CurrentLimits=pivotCurrentLimit;
    // motion magic setup (from Elevator subsystem) -- not used yet in algae manipulator code, ADD IF NECESSARY
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = 110; // Using the existing velocity value
    motionMagicConfigs.MotionMagicAcceleration = 190; // Using the existing acceleration value
    motionMagicConfigs.MotionMagicJerk = 1900; // Setting jerk to 10x acceleration as a starting point
    pivotConfigs.MotionMagic = motionMagicConfigs;

    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0; // Keeping the existing value
    slot0Configs.kP = 2; // Keeping the existing value
    slot0Configs.kI = 0; // Keeping the existing value
    slot0Configs.kD = 0.01; // Keeping the existing value
    pivotConfigs.Slot0 = slot0Configs;
    pivot.getConfigurator().apply(pivotConfigs);

    pivot.setPosition(pivotAbsoluteEncoder.get()+Constants.pivotAbsoluteEncoderOffset);

    intake = new TalonFX(Constants.CANIds.intakeMotor);
    laserCanCamera = new LaserCan(Constants.CANIds.laserCanCamera);
    TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();

    intakeCurrentLimit.StatorCurrentLimit=60;
    intakeCurrentLimit.StatorCurrentLimitEnable=true;
    intakeConfigs.CurrentLimits=intakeCurrentLimit;
    intake.getConfigurator().apply(intakeConfigs);

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pivot.setControl(motionMagicRequest.withPosition(pivotTargetPositionRotations).withSlot(0));
    intake.setControl(algaeIntakeDutyCycle.withOutput(intakeSpeed));
  }
  /**
   * Sets the target setPoint of the elevator.
   * @param setpoint Target position enum.
   */
  public void setPivotSetpoint(pivotPositions setpoint){
    pivotTargetPositionRotations = setpoint.getDegrees()/360;
  }


  public void setintakeSpeed(double intakeSpeed){
    this.intakeSpeed=intakeSpeed;
  }

  /**
   * Detects if a algae is present in the intake based on current draw.
   * @return true if a algae is detected (current exceeds threshold), false otherwise.
   */
  public boolean hasAlgaeLoaded() {
    return intake.getStatorCurrent().getValueAsDouble() > 34;
  }

  public boolean hasCoralLoaded() {
    return laserCanCamera.getMeasurement()==null;
  }

  public boolean isAlgaePivotCorrectPos(){
    return pivot.getPosition().getValueAsDouble()==pivotTargetPositionRotations;
  }

}
