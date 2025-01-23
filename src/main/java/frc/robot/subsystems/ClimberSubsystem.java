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

public class ClimberSubsystem extends SubsystemBase {
  public final TalonFX climberMotorLeader;
  public final TalonFX climberMotorFollower;
  public final TalonFX alignmentMotor;
  private final DutyCycleEncoder climberAbsoluteEncoder;

  private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private double climberTargetPositionRotations = 0;
  private double climberTargetPositionDegrees = 0;
  private double previousTargetPosition = climberTargetPositionRotations;

  private final CurrentLimitsConfigs climberCurrentLimits = new CurrentLimitsConfigs();
  private final CurrentLimitsConfigs alignmentCurrentLimit= new CurrentLimitsConfigs();

  private static final double CLIMBER_DEAD_ZONE = 1;
  
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem()
  {
    climberMotorLeader = new TalonFX(Constants.climberLeader);
    climberMotorFollower = new TalonFX(Constants.climberLeader);
    alignmentMotor = new TalonFX(Constants.climberAllignment);
    climberAbsoluteEncoder = new DutyCycleEncoder(Constants.climberAbsoluteEncoder);
    // climber motor follower is inverted
    climberMotorFollower.setControl(new Follower(Constants.climberLeader, true));

    // create a configuration object for the climber motor
    TalonFXConfiguration climberConfig = new TalonFXConfiguration();
    TalonFXConfiguration alignmentConfig = new TalonFXConfiguration();
    
    //climber configs:
    // current limits
    climberCurrentLimits.StatorCurrentLimit = 60;
    climberCurrentLimits.StatorCurrentLimitEnable = true;
    
    // apply the configuration to the climber motor
    climberConfig.CurrentLimits = climberCurrentLimits;
    
    var climberSlot0Configs = new Slot0Configs();
    climberSlot0Configs.kS = 0; // Keeping the existing value
    climberSlot0Configs.kP = 2; // Keeping the existing value
    climberSlot0Configs.kI = 0; // Keeping the existing value
    climberSlot0Configs.kD = 0.01; // Keeping the existing value

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
    allignmentSlot0Configs.kS = 0; // Keeping the existing value
    allignmentSlot0Configs.kP = 2; // Keeping the existing value
    allignmentSlot0Configs.kI = 0; // Keeping the existing value
    allignmentSlot0Configs.kD = 0.01; // Keeping the existing value
    climberMotorLeader.setPosition(0);
    alignmentConfig.Slot0 = allignmentSlot0Configs;

    climberMotorLeader.getConfigurator().apply(climberConfig);
    alignmentMotor.getConfigurator().apply(alignmentConfig);

    climberMotorLeader.setPosition(climberAbsoluteEncoder.get()+Constants.climberEncoderOffset);
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run

    executeClimberMotionMagic();
    
  }

  private void executeClimberMotionMagic() {
    motionMagicRequest.Position = climberTargetPositionRotations;
    motionMagicRequest.Slot = 0; // Use the Slot0 gains
    climberMotorLeader.setControl(motionMagicRequest);
  }



  /**
   * Sets the target position of the climber.
   * @param position Target position in rotations.
   */
  public void setClimberTargetPosition(double angle){

  }
  
    /**
     * Gets the current angle of the climber in degrees.
     * @return The current angle in degrees.
     */
    public double getClimberDegrees() {
      return climberMotorLeader.getPosition().getValueAsDouble() * 360*Constants.PhysicalConstants.ClimberReduction;
    }

    public boolean isClimberRightPosition(){
      return Math.abs(climberTargetPositionRotations-climberMotorLeader.getPosition().getValueAsDouble())<CLIMBER_DEAD_ZONE;
    }

}
