// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.data.Constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj.TimedRobot;

public class Manipulator extends SubsystemBase {
  /** Creates a new CoralIntake. */
  private TalonFX intake;
  private LaserCan laserCanCamera;
  private final CurrentLimitsConfigs intakeCurrentLimit= new CurrentLimitsConfigs();
  private double intakeSpeed = 0;
  
  public Manipulator() {
    // talonFX configs
    intake=new TalonFX(Constants.intakeMotor);
    laserCanCamera = new LaserCan(Constants.laserCanCamera);
    TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
    intakeCurrentLimit.StatorCurrentLimit=60;
    intakeCurrentLimit.StatorCurrentLimitEnable=true;
    intakeConfigs.CurrentLimits=intakeCurrentLimit;
    intake.getConfigurator().apply(intakeConfigs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final DutyCycleOut algaeIntakeDutyCycle = new DutyCycleOut(0);
    intake.setControl(algaeIntakeDutyCycle.withOutput(intakeSpeed));
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
}
