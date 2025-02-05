// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.data.Constants;
import frc.robot.utils.NetworkUser;
import frc.robot.utils.SubsystemNetworkManager;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

/**
 * The Funnel subsystem is responsible for pivoting the funnel
 * It controls a single pivot motor. 
 */
public class Funnel extends SubsystemBase implements NetworkUser {
  // Hardware Components
  public final TalonFX funnelPivotMotor;

  // Instance Variables
  private double funnelAngleSetpoint = 0;
  private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  
  // Constants
  private static final double FUNNEL_DEAD_ZONE = 1; // In degrees

  // Networktables Variables 
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable funnelTable = inst.getTable("Funnel");

  private final DoublePublisher funnelPivotSetpointNT = funnelTable.getDoubleTopic("Setpoint (Degrees)").publish();
  private final DoublePublisher funnelPivotAngleNT = funnelTable.getDoubleTopic("Current Angle (Degrees)").publish();

  /** Creates a new funnelSubsystem. */
  public Funnel() {
    SubsystemNetworkManager.RegisterNetworkUser(this);

    // Initialize hardware
    funnelPivotMotor = new TalonFX(Constants.CANIds.funnelMotor);

    // Configure hardware
    configurePivotMotor();
  }

  private void configurePivotMotor() {
    // create a configuration object for the funnel motor
    TalonFXConfiguration funnelConfig = new TalonFXConfiguration();

    // Current Limits
    CurrentLimitsConfigs funnelCurrentLimits = new CurrentLimitsConfigs();
    funnelCurrentLimits.StatorCurrentLimit = 60;
    funnelCurrentLimits.StatorCurrentLimitEnable = true;

    funnelConfig.CurrentLimits = funnelCurrentLimits;
    
    // PID Gains
    var funnelSlot0Configs = new Slot0Configs();
    funnelSlot0Configs.kS = 0; 
    funnelSlot0Configs.kP = 2; 
    funnelSlot0Configs.kI = 0; 
    funnelSlot0Configs.kD = 0.01; 

    funnelConfig.Slot0 = funnelSlot0Configs;

    // Motion Magic
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = 110; // Using the existing velocity value
    motionMagicConfigs.MotionMagicAcceleration = 190; // Using the existing acceleration value
    motionMagicConfigs.MotionMagicJerk = 1900; // Setting jerk to 10x acceleration as a starting point
    funnelConfig.MotionMagic = motionMagicConfigs;

    // Configure Mechanism Reduction
    funnelConfig.Feedback.SensorToMechanismRatio = Constants.PhysicalConstants.funnelReduction;

    // Apply Configuration
    funnelPivotMotor.getConfigurator().apply(funnelConfig);
  }

  @Override
  public void periodic() {
    funnelPivotMotor.setControl(motionMagicRequest.withPosition(funnelAngleSetpoint / 360).withSlot(0));
  }

  /**
   * Sets the angle setpoint
   * 
   * @param setpoint the setpoint in degrees
   */
  public void setFunnelSetpoint(double setpoint) {
    funnelAngleSetpoint = setpoint;
  }

  /**
   * Gets the current angle of the funnel in degrees.
   * 
   * @return The current angle in degrees.
   */
  public double getfunnelDegrees() {
    return funnelPivotMotor.getPosition().getValueAsDouble() * 360;
  }

  /**
   * Checks if a the funnel pivot is within a deadband of the desired setpoint
   * @return a boolean
   */
  public boolean isFunnelAtSetpoint() {
    return Math.abs(funnelAngleSetpoint - getfunnelDegrees()) < FUNNEL_DEAD_ZONE;
  }

  /* Networktables methods */

  /**
     * This method is called automatically by the SubsystemNetworkManager
     */
    @Override
    public void updateNetwork() {
      funnelPivotSetpointNT.set(funnelAngleSetpoint);
      funnelPivotAngleNT.set(getfunnelDegrees());
    }

  public void initializeNetwork() {
    // Could be used to make shuffleboard layouts programatically
    // Currently unused
  }
}

