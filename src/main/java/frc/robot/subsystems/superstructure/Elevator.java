// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.CANIds;
import frc.robot.data.Constants.ElevatorConstants;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.utils.PhoenixHelpers;
import frc.robot.utils.SecondOrderSim;
import frc.robot.utils.IO.TalonFXIO;
import frc.robot.utils.lib.WafflesMechanism;

public class Elevator extends WafflesMechanism {
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
  }

  // Hardware Components
  private final TalonFXIO elevatorMotorLeader;
  private final TalonFXIO elevatorMotorFollower;

  private SecondOrderSim elevatorSim;

  // Instance Variables
  private Trigger zeroingDebounceTrigger;
  private boolean isZeroingElevator = false;
  
  private SuperstructureState currentSetpointEnum = SuperstructureState.ZERO; 
  private CollisionType currentCollisionPrediction = CollisionType.NONE;
  private CollisionType potentialCollisionPrediction = CollisionType.NONE; // If the movement could induce collision

  private MotionMagicExpoVoltage motionMagicRequest = new MotionMagicExpoVoltage(0);

  // Networktables Variables 
  private final DoublePublisher elevatorPositionNT = networkTable.getDoubleTopic("Current Position (Meters)").publish();
  private final DoublePublisher elevatorVelocityNT = networkTable.getDoubleTopic("Current Velocity (rps)").publish();
  private final BooleanPublisher elevatorIsZeroingNT = networkTable.getBooleanTopic("Is Zeroing").publish();
  private final BooleanPublisher isAtSetpointNT = networkTable.getBooleanTopic("Elevator at Setpoint").publish();
  private final DoublePublisher leaderCurrentDrawNT = networkTable.getDoubleTopic("Leader Motor Current (Amps)").publish();
  private final DoublePublisher followerCurrentDrawNT = networkTable.getDoubleTopic("Follower Motor Current (Amps)").publish();
  
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
    elevatorMotorLeader = new TalonFXIO(CANIds.elevator1);
    elevatorMotorFollower = new TalonFXIO(CANIds.elevator2);

    configureElevatorMotors();

    zeroingDebounceTrigger = new Trigger(() -> {
      return elevatorMotorLeader.signals().statorCurrent().getValueAsDouble() > ElevatorConstants.STALL_CURRENT_THRESHOLD;   
    }).debounce(ElevatorConstants.ZERO_DEBOUNCE_TIME);

    if (RobotBase.isSimulation()) {
      elevatorSim = new SecondOrderSim(1.5, 1, 0, 0);
    }
  }

  /**
   * Configures both elevator motors. One is made a follower of the other.
   */
  private void configureElevatorMotors() {
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

    // Current Limits
    CurrentLimitsConfigs elevatorCurrentLimits = new CurrentLimitsConfigs();
    elevatorCurrentLimits.StatorCurrentLimit = 50;
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
    motionMagicConfigs.MotionMagicExpo_kV = 3; // kV is V/rps
    motionMagicConfigs.MotionMagicExpo_kA = 0.7; // Use a slower kA V/(rps/s)
    elevatorConfig.MotionMagic = motionMagicConfigs;

    // Mechanism Reduction
    elevatorConfig.Feedback.SensorToMechanismRatio = PhysicalConstants.elevatorReductionToMeters;

    elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Apply Configurations
    PhoenixHelpers.tryConfig(() -> elevatorMotorLeader.getConfigurator().apply(elevatorConfig));
    PhoenixHelpers.tryConfig(() -> elevatorMotorFollower.getConfigurator().apply(elevatorConfig));

    // Make Follower Motor
    elevatorMotorFollower.setControl(new Follower(CANIds.elevator1, false));
  }


  @Override
  public void periodicImpl() {
    if (isZeroingElevator) {
      handleElevatorZeroPeriodic();
      return;
    }

    // If elevator is high enough to use first stage, apply feedforward
    // Not needed for carriage only motion due to CF springs
    // double chosenFeedforward = 0;
    // if (getElevatorPositionMeters() > ElevatorConstants.FIRST_STAGE_START_HEIGHT) {
    //   chosenFeedforward = ElevatorConstants.kG;
    // }
    // double chosenFeedforward = ElevatorConstants.kG; // withFeedForward(chosenFeedforward)

    // Apply chosen setpoint
    elevatorMotorLeader.setControl(motionMagicRequest.withPosition(constrainedSetpoint).withSlot(0));
  }

  @Override
  protected void applyConstraints() {
    // Updated always so pivot always gets accurate information
    currentCollisionPrediction = isCollisionPredicted(setpoint);

    runConstraint(crossbarCollisionConstraint(), "Crossbar Collision");
    runConstraint(groundIntakeCollisionConstraint(), "Ground Intake Collision");
    runConstraint(mechanismLimitsConstraint(), "Mechanism Limits");
  }

  /**
   * Sets the target position of the elevator.
   * @param setpoint Target position (either SuperStructureState enum or height in meters)
   */
  public void applySetpoint(SuperstructureState setpoint) {
    applySetpoint(setpoint.getElevatorHeight());
    currentSetpointEnum = setpoint;    
  }

  /**
   * Get the last defined setpoint the elevator was set to
   * @return
   */
  public SuperstructureState getElevatorSetpointEnum(){
    return currentSetpointEnum;
  }

  /**
   * Gets the current elevator position in meters.
   * @return The current elevator position in meters.
   */
  public double getElevatorPositionMeters(){
    return elevatorMotorLeader.signals().position().getValueAsDouble();
  }

  /**
   * Gets a percentage value for how extended the elevator is
   * @return a 0-1 double representing percentage
   */
  public double getElevatorExtendedPercent() {
    return MathUtil.clamp( 
      getElevatorPositionMeters() / ElevatorConstants.MAX_ELEVATOR_HEIGHT,
      0, 1
    );
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
  @Override
  public boolean atSetpoint() {
    return Math.abs(getElevatorPositionMeters() - setpoint) < ElevatorConstants.ELEVATOR_DEAD_ZONE;
  }

  /**
   * Gets the current draw from the leader motor.
   * @return The leader motor's current draw in amps
   */
  public double getLeaderCurrent() {
    return elevatorMotorLeader.signals().statorCurrent().getValueAsDouble();
  }

  /**
   * Gets the current draw from the follower motor.
   * @return The follower motor's current draw in amps
   */
  public double getFollowerCurrent() {
    return elevatorMotorFollower.signals().statorCurrent().getValueAsDouble();
  }

  /*             */
  /* Constraints */
  /*             */

  private double mechanismLimitsConstraint() {
    return MathUtil.clamp(constrainedSetpoint, ElevatorConstants.MIN_ELEVATOR_HEIGHT, ElevatorConstants.MAX_ELEVATOR_HEIGHT);
  }

  private double groundIntakeCollisionConstraint() {
    if (RobotContainer.superstructure.pivot.getSetpoint() < ElevatorConstants.PIVOT_HITS_GROUND_INTAKE_ANGLE &&
      constrainedSetpoint < ElevatorConstants.GROUND_INTAKE_SAFETY_HEIGHT
    ) {
      return ElevatorConstants.GROUND_INTAKE_SAFETY_HEIGHT;
    }
    return constrainedSetpoint;
  }

  private double crossbarCollisionConstraint() {
    if (currentCollisionPrediction == CollisionType.NONE) {
      // Safe to move elevator
      // Move elevator to setpoint
      return constrainedSetpoint;

    } else if(currentCollisionPrediction == CollisionType.ENTERING_FROM_ABOVE) {
      // Move to safe setpoint
      return ElevatorConstants.COLLISION_ZONE_UPPER;

    } else if(currentCollisionPrediction == CollisionType.ENTERING_FROM_BELOW) {
      // Move to safe setpoint
      return ElevatorConstants.COLLISION_ZONE_LOWER;

    }

    // Try to stop motor in place
    return getElevatorPositionMeters();
  }

  /**
   * Checks if the elevator movement would cause a collision and what type of collision it would be
   * @param setpoint The target position the elevator is trying to move to in meters
   * @return The type of collision predicted, or NONE if movement is safe or pivot is in safe position
   */
  public CollisionType isCollisionPredicted(double setpoint) {
    potentialCollisionPrediction = predictedPotentialCollision(setpoint);
    
    // Check if pivot is in safe position
    boolean pivotSafe = RobotContainer.superstructure.pivot.getPivotPosition() > ElevatorConstants.CROSSBAR_MIN_CLEAR_ANGLE;
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


  /*             */
  /*   Network   */
  /*             */
  
  @Override
  public void updateNetwork() {
    elevatorPositionNT.set(getElevatorPositionMeters());
    elevatorIsZeroingNT.set(isZeroingElevator);
    isAtSetpointNT.set(atSetpoint());
    leaderCurrentDrawNT.set(elevatorMotorLeader.signals().statorCurrent().getValueAsDouble());
    followerCurrentDrawNT.set(elevatorMotorFollower.signals().statorCurrent().getValueAsDouble());
    elevatorVelocityNT.set(elevatorMotorLeader.signals().velocity().getValueAsDouble());
  }
  
  /*             */
  /*   Zeroing   */
  /*             */
  
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
  
  /**
   * Run periodically while zeroing elevator
   */
  private void handleElevatorZeroPeriodic() {
    if (zeroingDebounceTrigger.getAsBoolean() && isZeroingElevator) {
      // Stop the elevator
      elevatorMotorLeader.set(0);

      // Set the current position as the new zero
      elevatorMotorLeader.setPosition(0);

      // Reset the target position
      setpoint = 0;

      isZeroingElevator = false;

      DriverStation.reportWarning("Elevator zeroed successfully", false);
    }
  }

  /*              */
  /*  Simulation  */
  /*              */

  @Override 
  public void simulationPeriodic() {
    var talonFXSim = elevatorMotorLeader.getSimState();

    var simResult = elevatorSim.Evaluate(constrainedSetpoint, 0.02);

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // WPILIB sim objects return mechanism position/velocity (after gear ratio)
    talonFXSim.setRawRotorPosition(-simResult.get(0) * PhysicalConstants.elevatorReductionToMeters);
    talonFXSim.setRotorVelocity(-simResult.get(1) * PhysicalConstants.elevatorReductionToMeters);
  }
}
