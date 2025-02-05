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

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.data.Constants;
import frc.robot.data.Constants.ElevatorConstants;
import frc.robot.utils.NetworkUser;

public class Elevator extends SubsystemBase implements NetworkUser {
  // Hardware Components
  private final TalonFX elevatorMotorLeader;
  private final TalonFX elevatorMotorFollower;

  // Instance Variables
  private double elevatorSetpointMeters = 0;
  private boolean isZeroingElevator = false;

  private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  // Networktables Variables 
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable elevatorTable = inst.getTable("Elevator");

  private final DoublePublisher elevatorSetpointNT = elevatorTable.getDoubleTopic("Setpoint (Meters)").publish();
  private final DoublePublisher elevatorPositionNT = elevatorTable.getDoubleTopic("Current Position (Meters)").publish();
  private final BooleanPublisher elevatorIsZeroingNT = elevatorTable.getBooleanTopic("Is Zeroing").publish();

  // -------------------- Tuning Code --------------------
  // private NetworkConfiguredPID networkPIDConfiguration = new NetworkConfiguredPID(getName(), this::updatePID);
  
  // public void updatePID() {
  //   var slot0Configs = new Slot0Configs();
  //   slot0Configs.kS = networkPIDConfiguration.getS(); // Static feedforward
  //   slot0Configs.kP = networkPIDConfiguration.getP(); 
  //   slot0Configs.kI = networkPIDConfiguration.getI(); 
  //   slot0Configs.kD = networkPIDConfiguration.getD(); 


  //   Elevator1.getConfigurator().apply(slot0Configs);
  //   Elevator2.getConfigurator().apply(slot0Configs);
  // }

  public Elevator() {
    elevatorMotorLeader = new TalonFX(Constants.CANIds.elevator1);
    elevatorMotorFollower = new TalonFX(Constants.CANIds.elevator2);

    configureElevatorMotors();
  }

  /**
   * Configures both elevator motors. One is made a follower of the other.
   */
  private void configureElevatorMotors() {
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

    // Current Limits
    CurrentLimitsConfigs elevatorCurrentLimits = new CurrentLimitsConfigs();
    elevatorCurrentLimits.StatorCurrentLimit = 40;
    elevatorCurrentLimits.StatorCurrentLimitEnable = true;

    elevatorConfig.CurrentLimits = elevatorCurrentLimits;
    
    // PID Gains
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = ElevatorConstants.kS;
    slot0Configs.kP = ElevatorConstants.kP;
    slot0Configs.kI = ElevatorConstants.kI;
    slot0Configs.kD = ElevatorConstants.kD;

    elevatorConfig.Slot0 = slot0Configs;

    // Motion Magic
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.MOTION_CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.MOTION_ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = ElevatorConstants.MOTION_JERK;
    elevatorConfig.MotionMagic = motionMagicConfigs;

    // Mechanism Reduction
    elevatorConfig.Feedback.SensorToMechanismRatio = Constants.PhysicalConstants.elevatorReductionToMeters;

    // Apply Configurations
    elevatorMotorLeader.getConfigurator().apply(elevatorConfig);
    elevatorMotorFollower.getConfigurator().apply(elevatorConfig);

    // Make Follower Motor
    elevatorMotorFollower.setControl(new Follower(Constants.CANIds.elevator1, true));
  }

  public void periodic() {
    if (!isZeroingElevator) {
      elevatorMotorLeader.setControl(motionMagicRequest.withPosition(elevatorSetpointMeters).withSlot(0));
    } else {
      zeroElevator();
    }
  
    if (elevatorMotorLeader.getStatorCurrent().getValueAsDouble() > ElevatorConstants.STALL_CURRENT_THRESHOLD && isZeroingElevator) {
      // Stop the elevator
      elevatorMotorLeader.set(0);

      // Set the current position as the new zero
      elevatorMotorLeader.setPosition(0);

      // Reset the target position
      elevatorSetpointMeters = 0;

      isZeroingElevator = false;

      DriverStation.reportWarning("Elevator zeroed successfully", false);
    }
  }

  /**
   * Sets the target position of the elevator.
   * @param setpoint Target position in rotations.
   */
  public void setElevatorSetpointMeters(double setpoint){
    elevatorSetpointMeters = setpoint;
  }

  /**
   * Sets the target position of the elevator.
   * @param setpoint Target position enum.
   */
  public void setElevatorSetpoint(ElevatorConstants.ElevatorLevel setpoint){
    setElevatorSetpointMeters(setpoint.getHeight());
  }

  /**
   * Gets the target position of the elevator.
   * @return the setpoint Target position in rotations.
   */
  public double getElevatorSetpointMeters(){
    return elevatorSetpointMeters;
  }

  /**
   * Gets the current elevator position in meters.
   * @return The current elevator position in meters.
   */
  public double getElevatorPositionMeters(){
    return elevatorMotorLeader.getPosition().getValueAsDouble();
  }

  /**
   * Checks if the elevator is at the desired position.
   * @return true if elevator is at desired position, false otherwise.
   */
  public boolean isElevatorAtSetpoint() {
    return Math.abs(getElevatorPositionMeters() - elevatorSetpointMeters) < ElevatorConstants.ELEVATOR_DEAD_ZONE;
  }

  /**
   * Begins zeroing the elevator.
   */
  public void zeroElevator() {
    // Drive elevator down slowly
    elevatorMotorLeader.set(ElevatorConstants.ZEROING_SPEED);
    isZeroingElevator = true;
  }
  

  /* Networktables methods */
  @Override
  public void initializeNetwork() {
    // Could be used to make shuffleboard layouts programatically
    // Currently unused
  }

  /**
   * This method is called automatically by the SubsystemNetworkManager
   */
  @Override
  public void updateNetwork() {
    elevatorSetpointNT.set(elevatorSetpointMeters);
    elevatorPositionNT.set(getElevatorPositionMeters());
    elevatorIsZeroingNT.set(isZeroingElevator);
  }
}
