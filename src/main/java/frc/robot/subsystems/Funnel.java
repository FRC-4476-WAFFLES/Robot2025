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

public class Funnel extends SubsystemBase implements NetworkUser {
  private static final double FUNNEL_DEAD_ZONE = 1; // In degrees

  public final TalonFX funnelPivotMotor;

  
  private double funnelAngleSetpoint = 0;
  
  /* Networktables Variables */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable funnelTable = inst.getTable("Funnel");

  private final DoublePublisher funnelPivotSetpointNT = funnelTable.getDoubleTopic("Setpoint (Degrees)").publish();
  private final DoublePublisher funnelPivotAngleNT = funnelTable.getDoubleTopic("Current Angle (Degrees)").publish();

  /** Creates a new funnelSubsystem. */
  public Funnel() {
    SubsystemNetworkManager.RegisterNetworkUser(this);

    funnelPivotMotor = new TalonFX(Constants.CANIds.funnelMotor);

    // create a configuration object for the funnel motor
    TalonFXConfiguration funnelConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs funnelCurrentLimits = new CurrentLimitsConfigs();
    
    funnelCurrentLimits.StatorCurrentLimit = 60;
    funnelCurrentLimits.StatorCurrentLimitEnable = true;
    
    // apply the configuration to the climber motor
    funnelConfig.CurrentLimits = funnelCurrentLimits;
    
    var funnelSlot0Configs = new Slot0Configs();
    funnelSlot0Configs.kS = 0; // Keeping the existing value
    funnelSlot0Configs.kP = 2; // Keeping the existing value
    funnelSlot0Configs.kI = 0; // Keeping the existing value
    funnelSlot0Configs.kD = 0.01; // Keeping the existing value

    funnelConfig.Slot0 = funnelSlot0Configs;

    // Configure MotionMagic
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = 110; // Using the existing velocity value
    motionMagicConfigs.MotionMagicAcceleration = 190; // Using the existing acceleration value
    motionMagicConfigs.MotionMagicJerk = 1900; // Setting jerk to 10x acceleration as a starting point
    funnelConfig.MotionMagic = motionMagicConfigs;

    // Configure gear reduction
    funnelConfig.Feedback.SensorToMechanismRatio = Constants.PhysicalConstants.funnelReduction;

    funnelPivotMotor.getConfigurator().apply(funnelConfig);
  }

  @Override
  public void periodic() {
    // funnelPivot.setControl(motionMagicRequest.withPosition(funnelAngleSetpoint / 360).withSlot(0));
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
  public boolean isfunnelAtSetpoint() {
    return Math.abs(funnelAngleSetpoint - getfunnelDegrees()) < FUNNEL_DEAD_ZONE;
  }

  /* Networktables methods */
  public void initializeNetwork() {
    // Could be used to make shuffleboard layouts programatically
    // Currently unused
  }

  public void updateNetwork() {
    funnelPivotSetpointNT.set(funnelAngleSetpoint);
    funnelPivotAngleNT.set(getfunnelDegrees());
  }
}
