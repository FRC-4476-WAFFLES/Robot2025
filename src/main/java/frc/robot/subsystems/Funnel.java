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

  // TODO
  // in/out motor

  public final TalonFX funnelPivot;
  private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private double funnelTargetPositionRotations = 0;
  private double funnelTargetPositionDegrees = 0;
  private double previousTargetPosition = funnelTargetPositionRotations;
  private final CurrentLimitsConfigs funnelCurrentLimits = new CurrentLimitsConfigs();
  private final double FUNNEL_DEAD_ZONE = 1;
  private double rotationPosition;
  
  /** Creates a new funnelSubsystem. */
  public Funnel()
  {
    funnelPivot = new TalonFX(Constants.funnelMotor);

    // create a configuration object for the funnel motor
    TalonFXConfiguration funnelConfig = new TalonFXConfiguration();
    
    //funnel configs:
    // current limits
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

    funnelPivot.getConfigurator().apply(funnelConfig);
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    executefunnelMotionMagic();
    
  }

  private void executefunnelMotionMagic() {
    motionMagicRequest.Position = funnelTargetPositionRotations;
    motionMagicRequest.Slot = 0; // Use the Slot0 gains
    funnelPivot.setControl(motionMagicRequest);
  }



  public void SetRotafunneltionPosition(double rotPos)
  {
    this.rotationPosition = rotPos;
  }

    /**
   * Sets the target position of the funnel.
   * @param position Target position in rotations.
   */
  public void setFunnelTargetPosition(double angle){
    if (Math.abs(angle - this.funnelTargetPositionDegrees) > 0.05){
      this.funnelTargetPositionDegrees = MathUtil.clamp(angle,0,90);
      this.funnelTargetPositionRotations = funnelTargetPositionDegrees* (Constants.PhysicalConstants.OVERALL_REDUCTION / 360);
      if(this.funnelTargetPositionRotations != this.previousTargetPosition){
        this.previousTargetPosition = this.funnelTargetPositionRotations;
      }
    }
  }
  
    /**
     * Gets the current angle of the funnel in degrees.
     * @return The current angle in degrees.
     */
    public double getfunnelDegrees() {
      return funnelPivot.getPosition().getValueAsDouble() * 360*Constants.PhysicalConstants.OVERALL_REDUCTION;
    }

    public boolean isfunnelRightPosition(){
      return Math.abs(funnelTargetPositionRotations-funnelPivot.getPosition().getValueAsDouble())<FUNNEL_DEAD_ZONE;
    }

}
