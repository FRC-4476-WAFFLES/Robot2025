// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.data.Constants;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.ElevatorConstants;
import frc.robot.data.Constants.FunnelConstants;
import frc.robot.data.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.utils.NetworkUser;
import frc.robot.utils.SubsystemNetworkManager;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.RobotContainer.*;

public class Elevator extends SubsystemBase implements NetworkUser {
  /**
   * Enum representing different types of potential collisions
   */
  public enum CollisionType {
    /** No collision predicted */
    NONE,
    /** Currently inside collision zone */
    IN_ZONE,
    /** Will enter collision zone from below */
    ENTERING_FROM_BELOW,
    /** Will enter collision zone from above */
    ENTERING_FROM_ABOVE,
    /** Funnel is up, preventing elevator movement */
    FUNNEL_BLOCKING
  }

  // Hardware Components
  private final TalonFX elevatorMotorLeader;
  private final TalonFX elevatorMotorFollower;

  // Instance Variables
  private double elevatorSetpointMeters = 0;
  private boolean isZeroingElevator = false;

  private MotionMagicExpoVoltage motionMagicRequest = new MotionMagicExpoVoltage(0);

  // Networktables Variables 
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable elevatorTable = inst.getTable("Elevator");

  private final DoublePublisher elevatorSetpointNT = elevatorTable.getDoubleTopic("Setpoint (Meters)").publish();
  private final DoublePublisher elevatorPositionNT = elevatorTable.getDoubleTopic("Current Position (Meters)").publish();
  private final DoublePublisher elevatorVelocityNT = elevatorTable.getDoubleTopic("Current Velocity (rps)").publish();
  private final BooleanPublisher elevatorIsZeroingNT = elevatorTable.getBooleanTopic("Is Zeroing").publish();
  private final BooleanPublisher isAtSetpointNT = elevatorTable.getBooleanTopic("Elevator at Setpoint").publish();
  private final DoublePublisher leaderCurrentDrawNT = elevatorTable.getDoubleTopic("Leader Motor Current (Amps)").publish();
  private final DoublePublisher followerCurrentDrawNT = elevatorTable.getDoubleTopic("Follower Motor Current (Amps)").publish();
  private final BooleanPublisher funnelBlockingElevatorNT = elevatorTable.getBooleanTopic("Funnel Blocking Elevator").publish();

  private ElevatorLevel currentSetpointEnum = ElevatorLevel.REST_POSITION; 
  private CollisionType currentCollisionPrediction = CollisionType.NONE;
  private CollisionType potentialCollisionPrediction = CollisionType.NONE; // If the movement could induce collision

  // -------------------- Tuning Code --------------------
  // private NetworkConfiguredPID networkPIDConfiguration = new NetworkConfiguredPID(getName(), this::updatePID);
  
  // /**
  //  * Updates the PID and Motion Magic configurations from network tables values.
  //  * This is called automatically when network table values change.
  //  */
  // public void updatePID() {
  //   var slot0Configs = new Slot0Configs();
  //   slot0Configs.kS = networkPIDConfiguration.getS(); // Static feedforward
  //   slot0Configs.kP = networkPIDConfiguration.getP(); 
  //   slot0Configs.kI = networkPIDConfiguration.getI(); 
  //   slot0Configs.kD = networkPIDConfiguration.getD(); 
  //   slot0Configs.kG = ElevatorConstants.kG;
  //   slot0Configs.GravityType = GravityTypeValue.Elevator_Static;


  //   elevatorMotorLeader.getConfigurator().apply(slot0Configs);
  //   elevatorMotorFollower.getConfigurator().apply(slot0Configs);

  //   MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
  //   motionMagicConfigs.MotionMagicCruiseVelocity = networkPIDConfiguration.getMotionMagicCruiseVelocity(); 
  //   motionMagicConfigs.MotionMagicAcceleration = networkPIDConfiguration.getMotionMagicAcceleration();
  //   motionMagicConfigs.MotionMagicJerk = networkPIDConfiguration.getMotionMagicJerk(); 

  //   elevatorMotorLeader.getConfigurator().apply(motionMagicConfigs);
  //   elevatorMotorFollower.getConfigurator().apply(motionMagicConfigs);

  //   System.out.println("Refreshing PID values from networktables for elevator");
  // }

  /* SysId routine for characterizing elevator. */
  public final SysIdRoutine m_sysIdRoutineElevator = new SysIdRoutine(
      new SysIdRoutine.Config(
          null,        // Use default ramp rate (1 V/s)
          Volts.of(3), // Use dynamic voltage of 7 V
          null,        // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdElevator_State", state.toString())
      ),
      new SysIdRoutine.Mechanism(
          volts -> setElevatorVolts(volts),
          null,
          this
      )
  );
  // SysID Boilerplate
  private VoltageOut sysIDRequest = new VoltageOut(0);
  private void setElevatorVolts(Voltage volts) {
    elevatorMotorLeader.setControl(sysIDRequest.withOutput(volts));
  }

  public Elevator() {
    SubsystemNetworkManager.RegisterNetworkUser(this, true, CodeConstants.SUBSYSTEM_NT_UPDATE_RATE);

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
    elevatorCurrentLimits.StatorCurrentLimit = 60;
    elevatorCurrentLimits.StatorCurrentLimitEnable = true;

    elevatorConfig.CurrentLimits = elevatorCurrentLimits;
    
    // PID Gains
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = ElevatorConstants.kS;
    slot0Configs.kP = ElevatorConstants.kP;
    slot0Configs.kI = ElevatorConstants.kI;
    slot0Configs.kD = ElevatorConstants.kD;
    slot0Configs.kG = ElevatorConstants.kG;
    // slot0Configs.kV = 4.0;
    slot0Configs.GravityType = GravityTypeValue.Elevator_Static;

    elevatorConfig.Slot0 = slot0Configs;

    // Motion Magic
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    // motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.MOTION_CRUISE_VELOCITY;
    // motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.MOTION_ACCELERATION;
    // motionMagicConfigs.MotionMagicJerk = ElevatorConstants.MOTION_JERK;

    motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
    motionMagicConfigs.MotionMagicExpo_kV = 4.5; // kV is V/rps
    motionMagicConfigs.MotionMagicExpo_kA = 0.9; // Use a slower kA V/(rps/s)
    elevatorConfig.MotionMagic = motionMagicConfigs;

    // Mechanism Reduction
    elevatorConfig.Feedback.SensorToMechanismRatio = Constants.PhysicalConstants.elevatorReductionToMeters;

    elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Apply Configurations
    elevatorMotorLeader.getConfigurator().apply(elevatorConfig);
    elevatorMotorFollower.getConfigurator().apply(elevatorConfig);

    // Make Follower Motor
    elevatorMotorFollower.setControl(new Follower(Constants.CANIds.elevator1, false));
  }

  /**
   * Checks if the funnel is in a position that would block elevator movement
   * @return true if funnel is blocking elevator movement
   */
  public boolean isFunnelBlockingElevator() { 
    return RobotContainer.funnelSubsystem.getFunnelDegrees() > Constants.FunnelConstants.FUNNEL_BLOCKING_THRESHOLD;
  }

  @Override
  public void periodic() {
    // Updated always so pivot always gets accurate information
    currentCollisionPrediction = isCollisionPredicted(elevatorSetpointMeters);

    // Check if funnel is blocking elevator movement
    boolean funnelBlocking = isFunnelBlockingElevator();
    funnelBlockingElevatorNT.set(funnelBlocking);

    if (isZeroingElevator) {
      handleElevatorZeroPeriodic();
    } else {
      // Main control logic
      double chosenElevatorPosition = elevatorSetpointMeters;

      if (funnelBlocking && elevatorSetpointMeters > ElevatorConstants.COLLISION_ZONE_LOWER) {
        // If funnel is up and we're trying to move above the collision zone, limit to collision zone lower
        chosenElevatorPosition = ElevatorConstants.COLLISION_ZONE_LOWER;
      } else if (currentCollisionPrediction == Elevator.CollisionType.NONE) {
        // Safe to move elevator
        // Move elevator to setpoint
        chosenElevatorPosition = elevatorSetpointMeters; // not necessary but sanity check I guess
      } else if(currentCollisionPrediction == Elevator.CollisionType.ENTERING_FROM_ABOVE) {
        // Move to safe setpoint
        chosenElevatorPosition = ElevatorConstants.COLLISION_ZONE_UPPER;
      } else if(currentCollisionPrediction == Elevator.CollisionType.ENTERING_FROM_BELOW) {
        // Move to safe setpoint
        chosenElevatorPosition = ElevatorConstants.COLLISION_ZONE_LOWER;
      } else {
        // Try to stop motor
        chosenElevatorPosition = getElevatorPositionMeters();
      }

      // If elevator is high enough to use first stage, apply feedforward
      // Not needed for carriage only motion due to CF springs
      // double chosenFeedforward = 0;
      // if (getElevatorPositionMeters() > ElevatorConstants.FIRST_STAGE_START_HEIGHT) {
      //   chosenFeedforward = ElevatorConstants.kG;
      // }
      // double chosenFeedforward = ElevatorConstants.kG; // withFeedForward(chosenFeedforward)

      // Apply chosen setpoint
      elevatorMotorLeader.setControl(motionMagicRequest.withPosition(chosenElevatorPosition).withSlot(0));
    }

    // Update network tables
    updateNetwork();
  }

  /**
   * Run periodically while zeroing elevator
   */
  private void handleElevatorZeroPeriodic() {
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
   * @param setpoint Target position in meters.
   */
  private void setElevatorSetpointMeters(double setpoint){
    elevatorSetpointMeters = setpoint;
  }

  /**
   * Sets the target position of the elevator.
   * Can take either a direct height value in meters or a predefined ElevatorLevel position.
   * When using an ElevatorLevel, it will also update the target position state.
   * @param setpoint Target position (either ElevatorLevel enum or height in meters)
   */
  public void setElevatorSetpoint(Object setpoint) {
    if (setpoint instanceof ElevatorConstants.ElevatorLevel) {
      ElevatorConstants.ElevatorLevel levelSetpoint = (ElevatorConstants.ElevatorLevel) setpoint;
      setElevatorSetpointMeters(levelSetpoint.getHeight());
      currentSetpointEnum = levelSetpoint;

    } else if (setpoint instanceof Double || setpoint instanceof Integer) {
      double heightSetpoint = ((Number) setpoint).doubleValue();
      setElevatorSetpointMeters(heightSetpoint);
      // When using direct meter values, we don't update currentSetpointEnum since it doesn't correspond to a predefined level
      // This is yucky but I do not have the will to make a system that figures out which enum to use if the value is within a certain epsilon
      // Just assume it'll always be used with a setpoint enum 

    } else {
      throw new IllegalArgumentException("Setpoint must be either an ElevatorLevel or a numeric height in meters");
    }
  }

  /**
   * Get the last defined setpoint the elevator was set to
   * @return
   */
  public ElevatorLevel getElevatorSetpointEnum(){
    return currentSetpointEnum;
  }

  /**
   * Gets the target position of the elevator.
   * @return the setpoint Target position in meters.
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
   * Gets if the elevator is in a state that it will hit itself
   * @return a CollisionType enum
   */
  public CollisionType getCurrentCollisionPrediction() {
    return currentCollisionPrediction;
  }

  /**
   * Gets if the elevator is making a motion that requires the pivot to move for safety
   * @return
   */
  public CollisionType getCurrentCollisionPotential() {
    return potentialCollisionPrediction;
  }

  /**
   * Checks if the elevator is at the desired position.
   * @return true if elevator is at desired position, false otherwise.
   */
  public boolean isElevatorAtSetpoint() {
    return Math.abs(getElevatorPositionMeters() - elevatorSetpointMeters) < ElevatorConstants.ELEVATOR_DEAD_ZONE;
  }

  /**
   * Checks if the elevator is currently performing it's zeroing routine
   * @return true if elevator zeroing
   */
  public boolean isZeroing() {
    return isZeroingElevator;
  }

  /**
   * Begins zeroing the elevator.
   */
  public void zeroElevator() {
    // Drive elevator down slowly
    if (isZeroingElevator) {
      // Allow the operator to cancel zeroing elevator by pressing button again, in case zeroing fails
      isZeroingElevator = false;
      elevatorMotorLeader.set(0);
      DriverStation.reportWarning("Elevator zeroing canceled", false);

      return;
    }

    elevatorMotorLeader.set(ElevatorConstants.ZEROING_SPEED);
    isZeroingElevator = true;
  }
  

  /* Networktables methods */
  /**
   * Initializes network tables. Could be used to make shuffleboard layouts programmatically.
   * Currently unused but required by NetworkUser interface.
   */
  @Override
  public void initializeNetwork() {
    // Could be used to make shuffleboard layouts programatically
    // Currently unused
  }

  /**
   * Updates network table values with current elevator state.
   * This method is called automatically by the SubsystemNetworkManager.
   */
  @Override
  public void updateNetwork() {
    elevatorSetpointNT.set(elevatorSetpointMeters);
    elevatorPositionNT.set(getElevatorPositionMeters());
    elevatorIsZeroingNT.set(isZeroingElevator);
    isAtSetpointNT.set(isElevatorAtSetpoint());
    leaderCurrentDrawNT.set(elevatorMotorLeader.getStatorCurrent().getValueAsDouble());
    followerCurrentDrawNT.set(elevatorMotorFollower.getStatorCurrent().getValueAsDouble());
    elevatorVelocityNT.set(elevatorMotorLeader.getVelocity().getValueAsDouble());
  }

  /**
   * Gets the current draw from the leader motor.
   * @return The leader motor's current draw in amps
   */
  public double getLeaderCurrent() {
    return elevatorMotorLeader.getStatorCurrent().getValueAsDouble();
  }

  /**
   * Gets the current draw from the follower motor.
   * @return The follower motor's current draw in amps
   */
  public double getFollowerCurrent() {
    return elevatorMotorFollower.getStatorCurrent().getValueAsDouble();
  }

  /**
   * Checks if the elevator movement would cause a collision and what type of collision it would be
   * @param setpoint The target position the elevator is trying to move to in meters
   * @return The type of collision predicted, or NONE if movement is safe or pivot is in safe position
   */
  public CollisionType isCollisionPredicted(double setpoint) {
    potentialCollisionPrediction = predictedPotentialCollision(setpoint);
    
    // Check if funnel is blocking elevator movement
    if (isFunnelBlockingElevator() && setpoint > ElevatorConstants.COLLISION_ZONE_LOWER) {
      return CollisionType.FUNNEL_BLOCKING;
    }
    
    // Check if pivot is in safe position
    boolean pivotSafe = RobotContainer.pivotSubsystem.getPivotPosition() > ElevatorConstants.MIN_ELEVATOR_PIVOT_ANGLE;
                        // RobotContainer.manipulatorSubsystem.getPivotSetpoint() > ElevatorConstants.MIN_ELEVATOR_PIVOT_ANGLE;

    // If pivot is safe, no collision possible
    if (pivotSafe) {
      return CollisionType.NONE;
    }
    
    return potentialCollisionPrediction;
  }

  /**
   * Checks if the elevator movement would cause a collision and what type of collision it would be
   * @param setpoint The target position the elevator is trying to move to in meters
   * @return The type of collision predicted
   */
  public CollisionType predictedPotentialCollision(double setpoint) {
    double currentPosition = getElevatorPositionMeters();
    
    // First check if we're currently in the collision zone
    if (currentPosition >= ElevatorConstants.COLLISION_ZONE_LOWER && 
        currentPosition <= ElevatorConstants.COLLISION_ZONE_UPPER) {
      return CollisionType.IN_ZONE;
    }
    
    // If moving up (setpoint > current)
    if (setpoint > currentPosition) {
      // Check if path intersects collision zone from below
      if (currentPosition <= ElevatorConstants.COLLISION_ZONE_LOWER && 
          setpoint >= ElevatorConstants.COLLISION_ZONE_LOWER) {
        return CollisionType.ENTERING_FROM_BELOW;
      }
    } else {
      // If moving down (setpoint < current)
      // Check if path intersects collision zone from above
      if (currentPosition >= ElevatorConstants.COLLISION_ZONE_UPPER && 
          setpoint <= ElevatorConstants.COLLISION_ZONE_UPPER) {
        return CollisionType.ENTERING_FROM_ABOVE;
      }
    }
    
    // If setpoint is in the zone
    if (setpoint >= ElevatorConstants.COLLISION_ZONE_LOWER && 
        setpoint <= ElevatorConstants.COLLISION_ZONE_UPPER) {
      if (setpoint > currentPosition) {
        return CollisionType.ENTERING_FROM_BELOW;
      } else {
        return CollisionType.ENTERING_FROM_ABOVE;
      }
    }
    
    return CollisionType.NONE;
  }
}
