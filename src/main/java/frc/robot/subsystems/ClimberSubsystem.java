// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.data.Constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX climberMotorLeader;
  private final TalonFX climberMotorFollower;

  private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  private double climberSetpointAngle = 0;

  private static final double CLIMBER_DEAD_ZONE = 1;
  
  public enum ClimberAngle {
    DeployedAngle(30),
    RetractedAngle(0);

    private final double angle;

    ClimberAngle(double angle) {
      this.angle = angle;
    }

    public double getAngle() {
      return angle;
    }
  }

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem()
  {
    climberMotorLeader = new TalonFX(Constants.CANIds.climberLeader);
    climberMotorFollower = new TalonFX(Constants.CANIds.climberLeader);
    // climber motor follower is inverted
    climberMotorFollower.setControl(new Follower(Constants.CANIds.climberLeader, true));

    // create a configuration object for the climber motor
    TalonFXConfiguration climberConfig = new TalonFXConfiguration();
    TalonFXConfiguration alignmentConfig = new TalonFXConfiguration();

    CurrentLimitsConfigs climberCurrentLimits = new CurrentLimitsConfigs();
    CurrentLimitsConfigs alignmentCurrentLimit= new CurrentLimitsConfigs();
    
    //climber configs:
    // current limits
    climberCurrentLimits.StatorCurrentLimit = 60;
    climberCurrentLimits.StatorCurrentLimitEnable = true;
    
    // apply the configuration to the climber motor
    climberConfig.CurrentLimits = climberCurrentLimits;
    
    var climberSlot0Configs = new Slot0Configs();
    climberSlot0Configs.kS = 0;
    climberSlot0Configs.kP = 2;
    climberSlot0Configs.kI = 0;
    climberSlot0Configs.kD = 0.01; 

    climberConfig.Slot0 = climberSlot0Configs;

    // Configure MotionMagic
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = 110; // Using the existing velocity value
    motionMagicConfigs.MotionMagicAcceleration = 190; // Using the existing acceleration value
    motionMagicConfigs.MotionMagicJerk = 1900; // Setting jerk to 10x acceleration as a starting point
    climberConfig.MotionMagic = motionMagicConfigs;
    
    
    //allignment configs:
    alignmentCurrentLimit.StatorCurrentLimit=60;
    alignmentCurrentLimit.StatorCurrentLimitEnable=true;
    
    // apply the configuration to the climber motor
    alignmentConfig.CurrentLimits = alignmentCurrentLimit;

    var allignmentSlot0Configs = new Slot0Configs();
    allignmentSlot0Configs.kS = 0;
    allignmentSlot0Configs.kP = 2;
    allignmentSlot0Configs.kI = 0;
    allignmentSlot0Configs.kD = 0.01; 
    climberMotorLeader.setPosition(0);
    alignmentConfig.Slot0 = allignmentSlot0Configs;

    climberMotorLeader.getConfigurator().apply(climberConfig);
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    climberMotorLeader.setControl(motionMagicRequest.withPosition(climberSetpointAngle).withSlot(0));
  }



  /**
   * Sets the target position of the climber.
   * @param position Target position in degrees.
   */
  public void setClimberSetpoint(double angle){
    climberSetpointAngle = angle;
  }

  /**
   * Sets the target position of the climber.
   * @param position Target position from an enum
   */
  public void setClimberSetpoint(ClimberAngle angle){
    climberSetpointAngle = angle.getAngle();
  }
  
  /**
   * Gets the current angle of the climber in degrees.
   * @return The current angle in degrees.
   */
  public double getClimberDegrees() {
    return climberMotorLeader.getPosition().getValueAsDouble() * 360 * Constants.PhysicalConstants.ClimberReduction;
  }

  /**
   * Gets if the current angle of the climber is within a an allowable deadzone of it's setpoint.
   * @return A boolean.
   */
  public boolean isClimberRightPosition(){
    return Math.abs(climberSetpointAngle - getClimberDegrees()) < CLIMBER_DEAD_ZONE;
  }
}
