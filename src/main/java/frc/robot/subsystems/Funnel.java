// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.data.Constants;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.ElevatorConstants;
import frc.robot.data.Constants.FunnelConstants;
import frc.robot.data.Constants.FunnelConstants.FunnelPosition;
import frc.robot.utils.NetworkUser;
import frc.robot.utils.SubsystemNetworkManager;

/**
 * The Funnel subsystem is responsible for pivoting the funnel
 * It controls a single pivot motor. 
 */
public class Funnel extends SubsystemBase implements NetworkUser {
  // Hardware Components
  public final TalonFX funnelPivotMotor;

  // Instance Variables
  private double funnelAngleSetpoint = 0;
  private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  
  // Networktables Variables 
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable funnelTable = inst.getTable("Funnel");

  private final DoublePublisher funnelPivotSetpointNT = funnelTable.getDoubleTopic("Setpoint (Degrees)").publish();
  private final DoublePublisher funnelPivotAngleNT = funnelTable.getDoubleTopic("Current Angle (Degrees)").publish();
  private final BooleanPublisher funnelAtSetpointNT = funnelTable.getBooleanTopic("At Setpoint").publish();
  private final BooleanPublisher funnelSafeToMoveNT = funnelTable.getBooleanTopic("Safe To Move").publish();

  /** Creates a new funnelSubsystem. */
  public Funnel() {
    SubsystemNetworkManager.RegisterNetworkUser(this, true, CodeConstants.SUBSYSTEM_NT_UPDATE_RATE);

    // Initialize hardware
    funnelPivotMotor = new TalonFX(Constants.CANIds.funnelMotor);

    // Configure hardware
    configurePivotMotor();
  }

  private void configurePivotMotor() {
    // create a configuration object for the funnel motor
    TalonFXConfiguration funnelConfig = new TalonFXConfiguration();

    // Current Limits
    CurrentLimitsConfigs funnelCurrentLimits = new CurrentLimitsConfigs();
    funnelCurrentLimits.StatorCurrentLimit = FunnelConstants.STATOR_CURRENT_LIMIT;
    funnelCurrentLimits.StatorCurrentLimitEnable = true;

    funnelConfig.CurrentLimits = funnelCurrentLimits;
    
    // PID Gains
    var funnelSlot0Configs = new Slot0Configs();
    funnelSlot0Configs.kS = FunnelConstants.kS;
    funnelSlot0Configs.kP = FunnelConstants.kP;
    funnelSlot0Configs.kI = FunnelConstants.kI;
    funnelSlot0Configs.kD = FunnelConstants.kD;

    funnelConfig.Slot0 = funnelSlot0Configs;

    // Motion Magic
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = FunnelConstants.MOTION_CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = FunnelConstants.MOTION_ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = FunnelConstants.MOTION_JERK;
    funnelConfig.MotionMagic = motionMagicConfigs;

    // Configure Mechanism Reduction
    // This is the ratio between motor rotations and mechanism rotations
    // For example, if the motor needs to rotate 10 times to rotate the funnel once,
    // the SensorToMechanismRatio would be 10.0
    // This allows us to use degrees directly as our control unit
    funnelConfig.Feedback.SensorToMechanismRatio = Constants.PhysicalConstants.funnelReduction;
    
    // Set neutral mode to brake
    funnelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    
    // Add voltage compensation
    funnelConfig.Voltage.PeakForwardVoltage = 12.0; // 12V compensation
    funnelConfig.Voltage.PeakReverseVoltage = -12.0;
    funnelConfig.Voltage.SupplyVoltageTimeConstant = 0.1;
    funnelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Apply Configuration
    funnelPivotMotor.getConfigurator().apply(funnelConfig);
    
    // Reset the position to zero at startup
    // This assumes the funnel is at its zero position when the robot starts
    funnelPivotMotor.setPosition(0);
  }

  /**
   * Checks if it's safe to move the funnel based on elevator position
   * @return true if safe to move
   */
  public boolean isSafeToMoveFunnel() {
    // Only allow funnel movement if elevator is below the collision zone
    return RobotContainer.elevatorSubsystem.getElevatorPositionMeters() < ElevatorConstants.COLLISION_ZONE_LOWER;
  }

  @Override
  public void periodic() {
    // Convert degrees to rotations for motion magic
    // Since we've set the SensorToMechanismRatio, we need to convert our
    // desired angle in degrees to rotations of the mechanism
    double targetRotations = funnelAngleSetpoint / 360.0;
    
    // Check if it's safe to move the funnel
    boolean safeToMove = isSafeToMoveFunnel();
    funnelSafeToMoveNT.set(safeToMove);
    
    // Only apply motion magic control if it's safe to move
    if (safeToMove) {
      // Apply motion magic control
      funnelPivotMotor.setControl(motionMagicRequest.withPosition(targetRotations).withSlot(0));
    } else {
      // If not safe, hold current position
      double currentPosition = funnelPivotMotor.getPosition().getValueAsDouble();
      funnelPivotMotor.setControl(motionMagicRequest.withPosition(currentPosition).withSlot(0));
    }
    
    // Update network tables
    funnelAtSetpointNT.set(isFunnelAtSetpoint());
  }

  /**
   * Sets the angle setpoint for the funnel
   * 
   * @param setpoint the setpoint in degrees
   */
  public void setFunnelSetpoint(double setpoint) {
    // Clamp the setpoint to valid range
    funnelAngleSetpoint = MathUtil.clamp(setpoint, FunnelConstants.FUNNEL_MIN_ANGLE, FunnelConstants.FUNNEL_MAX_ANGLE);
  }

  /**
   * Sets the funnel position using a predefined FunnelPosition enum
   * 
   * @param position The FunnelPosition enum value
   */
  public void setFunnelPosition(FunnelPosition position) {
    setFunnelSetpoint(position.getDegrees());
  }

  /**
   * Gets the current angle of the funnel in degrees.
   * 
   * @return The current angle in degrees.
   */
  public double getFunnelDegrees() {
    // Get the position in rotations and convert to degrees
    // The SensorToMechanismRatio is automatically applied by the Phoenix library
    return funnelPivotMotor.getPosition().getValueAsDouble() * 360.0;
  }

  /**
   * Checks if the funnel pivot is within a deadband of the desired setpoint
   * @return true if funnel is at setpoint
   */
  public boolean isFunnelAtSetpoint() {
    return Math.abs(funnelAngleSetpoint - getFunnelDegrees()) < FunnelConstants.FUNNEL_DEAD_ZONE;
  }

  /**
   * Gets the current funnel setpoint
   * @return Current setpoint angle in degrees
   */
  public double getFunnelSetpoint() {
    return funnelAngleSetpoint;
  }

  /* Networktables methods */

  /**
   * This method is called automatically by the SubsystemNetworkManager
   */
  @Override
  public void updateNetwork() {
    funnelPivotSetpointNT.set(funnelAngleSetpoint);
    funnelPivotAngleNT.set(getFunnelDegrees());
  }

  public void initializeNetwork() {
    // Could be used to make shuffleboard layouts programatically
    // Currently unused
  }
}

