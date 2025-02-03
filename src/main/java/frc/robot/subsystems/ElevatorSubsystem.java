// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controls;
import frc.robot.data.Constants;
import frc.robot.utils.NetworkConfiguredPID;
import frc.robot.utils.NetworkUser;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

public class ElevatorSubsystem extends SubsystemBase implements NetworkUser {
  /** Creates a new ElevatorSubsystem. */
  private final TalonFX Elevator1;
  private final TalonFX Elevator2;

  private double elevatorSetpointMeters = 0;
  private boolean isZeroingElevator = false;

  private static final double ELEVATOR_DEAD_ZONE = 1;
  private static final double ZEROING_SPEED = -0.1; // Slow downward speed
  private static final double STALL_CURRENT_THRESHOLD = 10.0; // Amperes

  private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private NetworkConfiguredPID networkPIDConfiguration = new NetworkConfiguredPID(getName(), this::updatePID);
  
  public void updatePID() {
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = networkPIDConfiguration.getS(); // Static feedforward
    slot0Configs.kP = networkPIDConfiguration.getP(); 
    slot0Configs.kI = networkPIDConfiguration.getI(); 
    slot0Configs.kD = networkPIDConfiguration.getD(); 


    Elevator1.getConfigurator().apply(slot0Configs);
    Elevator2.getConfigurator().apply(slot0Configs);
  }

  public enum ElevatorLevel {
    //CHANGE VALUES!
    REST_POSITION(0),
    NET(90),
    ALGAE_L2(70),
    ALGAE_L1(60),
    PROCESSOR(55),
    CORAL_INTAKE(40),
    L3(50.0),
    L2(27.0),
    L1(10.0),
    L0(2);

    private double height;

    ElevatorLevel(double height) {
      this.height = height;
    }

    public double getHeight() {
      return height;
    }
  }

  /* Networktables Variables */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable elevatorTable = inst.getTable("Elevator");

  private final DoublePublisher elevatorSetpointNT = elevatorTable.getDoubleTopic("Setpoint (Meters)").publish();
  private final DoublePublisher elevatorPositionNT = elevatorTable.getDoubleTopic("Current Position (Meters)").publish();
  private final BooleanPublisher elevatorIsZeroingNT = elevatorTable.getBooleanTopic("Is Zeroing").publish();

  public ElevatorSubsystem() {
    Elevator1 = new TalonFX(Constants.CANIds.elevator1);
    Elevator2 = new TalonFX(Constants.CANIds.elevator2);
    Elevator2.setControl(new Follower(Constants.CANIds.elevator1, true));

    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs elevatorCurrentLimits = new CurrentLimitsConfigs();
    // elevatorCurrentLimits.SupplyCurrentLimit = 40;
    // elevatorCurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorCurrentLimits.StatorCurrentLimit = 40;
    elevatorCurrentLimits.StatorCurrentLimitEnable = true;

    elevatorConfig.CurrentLimits = elevatorCurrentLimits;
    
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0; // Keeping the existing value
    slot0Configs.kP = 2; // Keeping the existing value
    slot0Configs.kI = 0; // Keeping the existing value
    slot0Configs.kD = 0.01; // Keeping the existing value

    elevatorConfig.Slot0 = slot0Configs;

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = 110; // Using the existing velocity value
    motionMagicConfigs.MotionMagicAcceleration = 190; // Using the existing acceleration value
    motionMagicConfigs.MotionMagicJerk = 1900; // Setting jerk to 10x acceleration as a starting point
    elevatorConfig.MotionMagic = motionMagicConfigs;

    elevatorConfig.Feedback.SensorToMechanismRatio = Constants.PhysicalConstants.elevatorReductionToMeters;

    Elevator1.getConfigurator().apply(elevatorConfig);
    Elevator2.getConfigurator().apply(elevatorConfig);
  }

  public void periodic() {
    Elevator1.setControl(motionMagicRequest.withPosition(elevatorSetpointMeters).withSlot(0));

    if (Elevator1.getStatorCurrent().getValueAsDouble() > STALL_CURRENT_THRESHOLD && isZeroingElevator) {
      // Stop the elevator
      Elevator1.set(0);

      // Set the current position as the new zero
      Elevator1.setPosition(0);

      // Reset the target position
      elevatorSetpointMeters = 0;

      isZeroingElevator = false;

      System.out.println("Elevator zeroed successfully");
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
  public void setElevatorSetpoint(ElevatorLevel setpoint){
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
    return Elevator1.getPosition().getValueAsDouble();
  }

  // /**
  //  * Converts rotations to inches.
  //  * @param rotations Number of rotations.
  //  * @return Equivalent distance in inches.
  //  */
  // public double rotationsToInches(double rotations){
  //  return (rotations/19.0625)*(1.625*Math.PI);
  // }

  // /**
  //  * Converts inches to rotations.
  //  * @param inches Distance in inches.
  //  * @return Equivalent number of rotations.
  //  */
  // public double inchesToRotations(double inches){
  //   return (inches*19.0625)/(1.625/Math.PI);
  // }

  /**
   * Checks if the elevator is at the desired position.
   * @return true if elevator is at desired position, false otherwise.
   */
  public boolean isElevatorAtSetpoint() {
    return Math.abs(getElevatorPositionMeters() - elevatorSetpointMeters) < ELEVATOR_DEAD_ZONE;
  }


  /**
   * Begins zeroing the elevator.
   */
  public void zeroElevator() {
    // Drive elevator down slowly
    Elevator1.set(ZEROING_SPEED);
    isZeroingElevator = true;
  }

  /* Networktables methods */
  public void initializeNetwork() {
    // Could be used to make shuffleboard layouts programatically
    // Currently unused
  }

  public void updateNetwork() {
    elevatorSetpointNT.set(elevatorSetpointMeters);
    elevatorPositionNT.set(getElevatorPositionMeters());
    elevatorIsZeroingNT.set(isZeroingElevator);
  }
}
