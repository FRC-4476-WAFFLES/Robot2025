// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.data.Constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

public class Funnel extends SubsystemBase {
  // one leader motor, one follower motor to align robot with cage
  // motor to lower arm, which holds the cage and lifts the robot
  // intake / outtake motor

  private static final double FUNNEL_DEAD_ZONE = 1;

  public final TalonFX funnelPivot;

  private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private double funnelAngleSetpoint = 0;

  /** Creates a new funnelSubsystem. */
  public Funnel()
  {
    funnelPivot = new TalonFX(Constants.funnelMotor);

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

    funnelPivot.getConfigurator().apply(funnelConfig);
  }

  @Override
  public void periodic() {
    funnelPivot.setControl(motionMagicRequest.withPosition(funnelAngleSetpoint / 360).withSlot(0));
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
    return funnelPivot.getPosition().getValueAsDouble() * 360;
  }

  public boolean isfunnelRightPosition() {
    return Math.abs(funnelAngleSetpoint - getfunnelDegrees()) < FUNNEL_DEAD_ZONE;
  }

}
