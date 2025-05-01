// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.data.Constants;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.SharkPivotConstants;
import frc.robot.data.Constants.SharkPivotConstants.SharkPivotPosition;
import frc.robot.utils.NetworkUser;
import frc.robot.utils.PhoenixHelpers;
import frc.robot.utils.SubsystemNetworkManager;
import frc.robot.utils.IO.TalonFXIO;

/**
 * The SharkPivot subsystem is responsible for pivoting the L1 Intake (Shark)
 * It controls a single pivot motor. 
 */
public class SharkPivot extends SubsystemBase implements NetworkUser {
  // Hardware Components
  public final TalonFXIO pivotMotor;

  // Instance Variables
  private double angleSetpoint = 0;
  private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private boolean isZeroingPivot = false;
  private Trigger zeroingDebounceTrigger;
  
  // Networktables Variables 
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable sharkPivotTable = inst.getTable("Shark Pivot");

  private final DoublePublisher sharkPivotSetpointNT = sharkPivotTable.getDoubleTopic("Setpoint (Degrees)").publish();
  private final DoublePublisher sharkPivotAngleNT = sharkPivotTable.getDoubleTopic("Current Angle (Degrees)").publish();
  private final BooleanPublisher sharkPivotAtSetpointNT = sharkPivotTable.getBooleanTopic("At Setpoint").publish();
  private final BooleanPublisher sharkPivotisZeroingNT = sharkPivotTable.getBooleanTopic("Is Zeroing").publish();

  // -------------------- Tuning Code --------------------
  // private NetworkConfiguredPID networkPIDConfiguration = new NetworkConfiguredPID(getName(), this::updatePID);
  
  // public void updatePID() {
  //   var slot0Configs = new Slot0Configs();
  //   slot0Configs.kS = networkPIDConfiguration.getS(); // Static feedforward
  //   slot0Configs.kP = networkPIDConfiguration.getP(); 
  //   slot0Configs.kI = networkPIDConfiguration.getI(); 
  //   slot0Configs.kD = networkPIDConfiguration.getD(); 

  //   pivotMotor.getConfigurator().apply(slot0Configs);

  //   MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
  //   motionMagicConfigs.MotionMagicCruiseVelocity = networkPIDConfiguration.getMotionMagicCruiseVelocity(); 
  //   motionMagicConfigs.MotionMagicAcceleration = networkPIDConfiguration.getMotionMagicAcceleration();
  //   motionMagicConfigs.MotionMagicJerk = networkPIDConfiguration.getMotionMagicJerk(); 

  //   pivotMotor.getConfigurator().apply(motionMagicConfigs);

  //   System.out.println("Refreshing PID values from networktables for shark pivot");
  // }


  /** Creates a new L1 Pivot Subsystem. */
  public SharkPivot() {
    SubsystemNetworkManager.RegisterNetworkUser(this, true, CodeConstants.SUBSYSTEM_NT_UPDATE_RATE);

    // Initialize hardware
    pivotMotor = new TalonFXIO(Constants.CANIds.sharkPivotMotor);

    // Configure hardware
    configurePivotMotor();

    zeroingDebounceTrigger = new Trigger(() -> {
      return pivotMotor.signals().torqueCurrent().getValueAsDouble() < -SharkPivotConstants.PIVOT_CURRENT_THRESHOLD;     
    }).debounce(SharkPivotConstants.ZERO_DEBOUNCE_TIME);
  }

  private void configurePivotMotor() {
    // create a configuration object for the pivot motor
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

    // Current Limits
    CurrentLimitsConfigs pivotCurrentLimits = new CurrentLimitsConfigs();
    pivotCurrentLimits.StatorCurrentLimit = SharkPivotConstants.STATOR_CURRENT_LIMIT;
    pivotCurrentLimits.StatorCurrentLimitEnable = true;

    pivotConfig.CurrentLimits = pivotCurrentLimits;
    
    // PID Gains
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = SharkPivotConstants.kS;
    slot0Configs.kP = SharkPivotConstants.kP;
    slot0Configs.kI = SharkPivotConstants.kI;
    slot0Configs.kD = SharkPivotConstants.kD;

    pivotConfig.Slot0 = slot0Configs;

    // Motion Magic
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = SharkPivotConstants.MOTION_CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = SharkPivotConstants.MOTION_ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = SharkPivotConstants.MOTION_JERK;
    pivotConfig.MotionMagic = motionMagicConfigs;

    // Configure Mechanism Reduction
    // This is the ratio between motor rotations and mechanism rotations
    // For example, if the motor needs to rotate 10 times to rotate the mechanism once,
    // the SensorToMechanismRatio would be 10.0
    // This allows us to use degrees directly as our control unit
    pivotConfig.Feedback.SensorToMechanismRatio = Constants.PhysicalConstants.sharkPivotReduction;
    
    // Set neutral mode to brake
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    
    // Add voltage compensation
    pivotConfig.Voltage.PeakForwardVoltage = 12.0; // 12V compensation
    pivotConfig.Voltage.PeakReverseVoltage = -12.0;
    pivotConfig.Voltage.SupplyVoltageTimeConstant = 0.1;
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Apply Configuration
    PhoenixHelpers.tryConfig(() -> pivotMotor.getConfigurator().apply(pivotConfig));
    
    // Reset the position to zero at startup
    // This assumes the pivot is at its zero position when the robot starts
    pivotMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    // Handle zeroing first
    if (isZeroingPivot) {
      handlePivotZeroPeriodic();
      return;
    }

    // Convert degrees to rotations for motion magic
    // Since we've set the SensorToMechanismRatio, we need to convert our
    // desired angle in degrees to rotations of the mechanism
    double targetRotations = angleSetpoint / 360.0;
    
    pivotMotor.setControl(motionMagicRequest.withPosition(targetRotations).withSlot(0));
    
    // Update network tables
    sharkPivotAtSetpointNT.set(isPivotAtSetpoint());
  }

  /**
   * Sets the angle setpoint for the shark
   * 
   * @param setpoint the setpoint in degrees
   */
  public void setPivotSetpoint(double setpoint) {
    // Clamp the setpoint to valid range
    angleSetpoint = MathUtil.clamp(setpoint, SharkPivotConstants.MIN_ANGLE, SharkPivotConstants.MAX_ANGLE);
  }

  /**
   * Sets the shark position using a predefined SharkPivot enum
   * 
   * @param position The SharkPivotPosition enum value
   */
  public void setPivotPosition(SharkPivotPosition position) {
    setPivotSetpoint(position.getDegrees());
  }

  /**
   * Gets the current angle of the shark in degrees.
   * 
   * @return The current angle in degrees.
   */
  public double getPivotDegrees() {
    // Get the position in rotations and convert to degrees
    // The SensorToMechanismRatio is automatically applied by the Phoenix library
    return pivotMotor.signals().position().getValueAsDouble() * 360.0;
  }

  /**
   * Checks if the shark pivot is within a deadband of the desired setpoint
   * @return true if shark is at setpoint
   */
  public boolean isPivotAtSetpoint() {
    return Math.abs(angleSetpoint - getPivotDegrees()) < SharkPivotConstants.DEAD_ZONE;
  }

  /**
   * Gets the current shark setpoint
   * @return Current setpoint angle in degrees
   */
  public double getSharkSetpoint() {
    return angleSetpoint;
  }

  /**
   * Run periodically while zeroing pivot
   */
  private void handlePivotZeroPeriodic() {
    if (zeroingDebounceTrigger.getAsBoolean()) {
      pivotMotor.set(0);
      pivotMotor.setPosition(0.0);
      setPivotSetpoint(0);
      
      isZeroingPivot = false;
      DriverStation.reportWarning("PivotShark zeroed successfully", false);
      
      return;
    }
    pivotMotor.set(SharkPivotConstants.ZEROING_SPEED);
  }

  /**
  * Begins zeroing the pivot.
  */
  public void zeroPivot() {
    if (isZeroingPivot) {
      isZeroingPivot = false;
      pivotMotor.set(0);
      DriverStation.reportWarning("PivotShark zeroing canceled", false);
          
      return;
    }

    // Cancel if called again
    isZeroingPivot = true;
  }

  /**
   * Checks if the pivot is currently performing its zeroing routine
   * @return true if pivot is zeroing
   */
  public boolean isZeroing() {
    return isZeroingPivot;
  }

  /* Networktables methods */

  /**
   * This method is called automatically by the SubsystemNetworkManager
   */
  @Override
  public void updateNetwork() {
    sharkPivotSetpointNT.set(angleSetpoint);
    sharkPivotAngleNT.set(getPivotDegrees());
    sharkPivotisZeroingNT.set(isZeroingPivot);
  }

  public void initializeNetwork() {
    // Could be used to make shuffleboard layouts programatically
    // Currently unused
  }
}

