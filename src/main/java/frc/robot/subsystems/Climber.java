// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.data.Constants;
import frc.robot.data.Constants.CANIds;
import frc.robot.data.Constants.ClimberConstants;
import frc.robot.data.Constants.ClimberConstants.ClimberPosition;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.utils.NetworkUser;
import frc.robot.utils.SubsystemNetworkManager;

/**
 * The Climber subsystem is responsible for controlling the robot's climbing mechanism.
 * It controls a leader motor and a follower motor for synchronized movement.
 */
public class Climber extends SubsystemBase implements NetworkUser {
  // Hardware Components
  private final TalonFX climberMotorLeader;
  private final TalonFX climberMotorFollower;

  // Instance Variables
  private double climberSetpointAngle = 0;
  private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  
  // Networktables Variables 
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable climberTable = inst.getTable("Climber");

  private final DoublePublisher climberSetpointNT = climberTable.getDoubleTopic("Setpoint (Degrees)").publish();
  private final DoublePublisher climberAngleNT = climberTable.getDoubleTopic("Current Angle (Degrees)").publish();
  private final BooleanPublisher climberAtSetpointNT = climberTable.getBooleanTopic("At Setpoint").publish();

  /** Creates a new Climber subsystem. */
  public Climber() {
    SubsystemNetworkManager.RegisterNetworkUser(this, true, CodeConstants.SUBSYSTEM_NT_UPDATE_RATE);

    // Initialize hardware
    climberMotorLeader = new TalonFX(CANIds.climberLeader);
    climberMotorFollower = new TalonFX(CANIds.climberFollower);

    // Configure hardware
    configureClimberMotors();
  }

  /**
   * Configures the climber motors with appropriate settings
   */
  private void configureClimberMotors() {
    // Create a configuration object for the climber motors
    TalonFXConfiguration climberConfig = new TalonFXConfiguration();

    // Current Limits
    CurrentLimitsConfigs climberCurrentLimits = new CurrentLimitsConfigs();
    climberCurrentLimits.StatorCurrentLimit = ClimberConstants.STATOR_CURRENT_LIMIT;
    climberCurrentLimits.StatorCurrentLimitEnable = true;

    climberConfig.CurrentLimits = climberCurrentLimits;
    
    // PID Gains
    var climberSlot0Configs = new Slot0Configs();
    climberSlot0Configs.kS = ClimberConstants.kS;
    climberSlot0Configs.kP = ClimberConstants.kP;
    climberSlot0Configs.kI = ClimberConstants.kI;
    climberSlot0Configs.kD = ClimberConstants.kD;

    climberConfig.Slot0 = climberSlot0Configs;

    // Motion Magic
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = ClimberConstants.MOTION_CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = ClimberConstants.MOTION_ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = ClimberConstants.MOTION_JERK;
    climberConfig.MotionMagic = motionMagicConfigs;

    // Configure Mechanism Reduction
    climberConfig.Feedback.SensorToMechanismRatio = Constants.PhysicalConstants.ClimberReduction;
    
    // Set neutral mode to brake
    climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    
    climberConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Add voltage compensation
    climberConfig.Voltage.PeakForwardVoltage = 12.0; // 12V compensation
    climberConfig.Voltage.PeakReverseVoltage = -12.0;
    climberConfig.Voltage.SupplyVoltageTimeConstant = 0.1;

    // Apply Configuration to leader
    climberMotorLeader.getConfigurator().apply(climberConfig);
    
    // Reset the position to zero at startup
    climberMotorLeader.setPosition(0);
    
    // Configure follower
    climberMotorFollower.getConfigurator().apply(climberConfig);
    climberMotorFollower.setControl(new Follower(CANIds.climberLeader, false));
  }

  @Override
  public void periodic() {
    // Convert degrees to rotations for motion magic
    double targetRotations = climberSetpointAngle / 360.0;
    
    // Apply motion magic control
    climberMotorLeader.setControl(motionMagicRequest.withPosition(targetRotations).withSlot(0));
    
    // Update network tables
    climberAtSetpointNT.set(isClimberAtSetpoint());
  }

  /**
   * Sets the angle setpoint for the climber
   * 
   * @param setpoint the setpoint in degrees
   */
  public void setClimberSetpoint(double setpoint) {
    // Clamp the setpoint to valid range
    climberSetpointAngle = MathUtil.clamp(setpoint, ClimberConstants.CLIMBER_MIN_ANGLE, ClimberConstants.CLIMBER_MAX_ANGLE);
  }

  /**
   * Sets the climber position using a predefined ClimberPosition enum
   * 
   * @param position The ClimberPosition enum value
   */
  public void setClimberPosition(ClimberPosition position) {
    setClimberSetpoint(position.getDegrees());
  }

  /**
   * Gets the current angle of the climber in degrees.
   * 
   * @return The current angle in degrees.
   */
  public double getClimberDegrees() {
    // Get the position in rotations and convert to degrees
    return climberMotorLeader.getPosition().getValueAsDouble() * 360.0;
  }

  /**
   * Checks if the climber is within a deadband of the desired setpoint
   * @return true if climber is at setpoint
   */
  public boolean isClimberAtSetpoint() {
    return Math.abs(climberSetpointAngle - getClimberDegrees()) < ClimberConstants.CLIMBER_DEAD_ZONE;
  }

  /**
   * Gets the current climber setpoint
   * @return Current setpoint angle in degrees
   */
  public double getClimberSetpoint() {
    return climberSetpointAngle;
  }

  /* Networktables methods */

  /**
   * This method is called automatically by the SubsystemNetworkManager
   */
  @Override
  public void updateNetwork() {
    climberSetpointNT.set(climberSetpointAngle);
    climberAngleNT.set(getClimberDegrees());
  }

  @Override
  public void initializeNetwork() {
    // Could be used to make shuffleboard layouts programatically
    // Currently unused
  }
}
