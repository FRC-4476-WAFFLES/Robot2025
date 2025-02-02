// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.data.Constants;
import frc.robot.utils.NetworkUser;
import frc.robot.utils.SubsystemNetworkManager;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import au.grapplerobotics.LaserCan;


public class Manipulator extends SubsystemBase implements NetworkUser {
  /** Creates a new Manipulator. */
  private final TalonFX intake;
  private final TalonFX pivot;
  private final DutyCycleEncoder pivotAbsoluteEncoder;
  private final LaserCan laserCanCamera;

  private final DutyCycleOut algaeIntakeDutyCycle = new DutyCycleOut(0);
  private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  public static final double CORAL_LOADED_DISTANCE_THRESHOLD = 30; // mm
  public static final double PIVOT_ANGLE_DEADBAND = 0.015; // rotations

  private double intakeSpeed = 0;
  private double pivotSetpointAngle = 0;

  /* Networktables Variables */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable pivotTable = inst.getTable("Pivot");

  private final DoublePublisher pivotSetpointNT = pivotTable.getDoubleTopic("Setpoint (Degrees)").publish();
  private final DoublePublisher pivotAngleNT = pivotTable.getDoubleTopic("Current Angle (Degrees)").publish();

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
    SubsystemNetworkManager.RegisterNetworkUser(this);

    pivot = new TalonFX(Constants.CANIds.pivotMotor);
    pivotAbsoluteEncoder = new DutyCycleEncoder(Constants.CANIds.pivotAbsoluteEncoder);

    TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
    CurrentLimitsConfigs pivotCurrentLimit= new CurrentLimitsConfigs();
    CurrentLimitsConfigs intakeCurrentLimit= new CurrentLimitsConfigs();

    pivotCurrentLimit.StatorCurrentLimit = 60;
    pivotCurrentLimit.StatorCurrentLimitEnable = true;
    pivotConfigs.CurrentLimits = pivotCurrentLimit;
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

    pivotConfigs.Feedback.SensorToMechanismRatio = Constants.PhysicalConstants.pivotReduction;

    pivot.getConfigurator().apply(pivotConfigs);

    // Init from absolute encoder
    pivot.setPosition(pivotAbsoluteEncoder.get() + Constants.PhysicalConstants.pivotAbsoluteEncoderOffset);

    intake = new TalonFX(Constants.CANIds.intakeMotor);
    TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();

    intakeCurrentLimit.StatorCurrentLimit = 60;
    intakeCurrentLimit.StatorCurrentLimitEnable = true;
    intakeConfigs.CurrentLimits = intakeCurrentLimit;
    intake.getConfigurator().apply(intakeConfigs); 

    laserCanCamera = new LaserCan(Constants.CANIds.laserCanCamera);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pivot.setControl(motionMagicRequest.withPosition(pivotSetpointAngle / 360).withSlot(0));
    intake.setControl(algaeIntakeDutyCycle.withOutput(intakeSpeed));
  }

  /**
   * Sets the target setPoint of the elevator.
   * @param setpoint Target position in degrees.
   */
  public void setPivotSetpoint(double setpoint){
    pivotSetpointAngle = setpoint;
  }

  /**
   * Sets the target setPoint of the elevator.
   * @param setpoint Target position enum.
   */
  public void setPivotSetpoint(pivotPositions setpoint){
    pivotSetpointAngle = setpoint.getDegrees();
  }

  /**
   * Gets the current pivot angle in degrees.
   * @return The current pivot angle in degrees.
   */
  public double getPivotPosition() {
    return pivot.getPosition().getValueAsDouble() * 360;
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

  /**
   * Checks if a coral is loaded with a laser distance reading
   * @return true if a coral is detected (sensor reports within a certain distance), false otherwise.
   */
  public boolean hasCoralLoaded() {
    var measurement = laserCanCamera.getMeasurement();

    if (measurement != null) {
      if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
        return measurement.distance_mm <= CORAL_LOADED_DISTANCE_THRESHOLD;
      }
    }

    return false; 
  }

  /**
   * Checks if a the pivot is within a deadband of the desired setpoint
   * @return a boolean
   */
  public boolean isPivotAtSetpoint(){
    return Math.abs(getPivotPosition() - pivotSetpointAngle) < PIVOT_ANGLE_DEADBAND;
  }

  /* Networktables methods */
  public void initializeNetwork() {
    // Could be used to make shuffleboard layouts programatically
    // Currently unused
  }

  public void updateNetwork() {
    pivotSetpointNT.set(pivotSetpointAngle);
    pivotAngleNT.set(getPivotPosition());
  }
}
