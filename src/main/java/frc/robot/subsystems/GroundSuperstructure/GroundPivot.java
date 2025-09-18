// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundsuperstructure;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.data.Constants;
import frc.robot.data.Constants.GroundPivotConstants;
import frc.robot.data.Constants.GroundPivotConstants.GroundPivotPosition;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.utils.PhoenixHelpers;
import frc.robot.utils.SecondOrderSim;
import frc.robot.utils.IO.TalonFXIO;
import frc.robot.utils.lib.WafflesMechanism;

/**
 * The GroundPivot subsystem is responsible for pivoting the L1 Intake 
 * It controls a single pivot motor. 
 */
public class GroundPivot extends WafflesMechanism {
  // Hardware Components
  public final TalonFXIO pivotMotor;

  private SecondOrderSim pivotSim;

  // Instance Variables
  private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private boolean isZeroingPivot = false;
  private Trigger zeroingDebounceTrigger;
  
  // Networktables Variables 
  private final DoublePublisher groundPivotAngleNT = networkTable.getDoubleTopic("Current Angle (Degrees)").publish();
  private final BooleanPublisher groundPivotAtSetpointNT = networkTable.getBooleanTopic("At Setpoint").publish();
  private final BooleanPublisher groundPivotisZeroingNT = networkTable.getBooleanTopic("Is Zeroing").publish();

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

  //   System.out.println("Refreshing PID values from networktables for ground pivot");
  // }


  /** Creates a new L1 Pivot Subsystem. */
  public GroundPivot() {
    // Initialize hardware
    pivotMotor = new TalonFXIO(Constants.CANIds.groundPivotMotor);

    // Configure hardware
    configurePivotMotor();

    zeroingDebounceTrigger = new Trigger(() -> {
      return pivotMotor.signals().torqueCurrent().getValueAsDouble() < -GroundPivotConstants.PIVOT_CURRENT_THRESHOLD
      && isZeroingPivot;     
    }).debounce(GroundPivotConstants.ZERO_DEBOUNCE_TIME);

    if (RobotBase.isSimulation()) {
      pivotSim = new SecondOrderSim(2.5, 1, 0, 0);
    }
  }

  /**
   * Configures the pivot motor
   */
  private void configurePivotMotor() {
    // create a configuration object for the pivot motor
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

    // Current Limits
    CurrentLimitsConfigs pivotCurrentLimits = new CurrentLimitsConfigs();
    pivotCurrentLimits.StatorCurrentLimit = GroundPivotConstants.STATOR_CURRENT_LIMIT;
    pivotCurrentLimits.StatorCurrentLimitEnable = true;

    pivotConfig.CurrentLimits = pivotCurrentLimits;
    
    // PID Gains
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = GroundPivotConstants.kS;
    slot0Configs.kP = GroundPivotConstants.kP;
    slot0Configs.kI = GroundPivotConstants.kI;
    slot0Configs.kD = GroundPivotConstants.kD;

    pivotConfig.Slot0 = slot0Configs;

    // Motion Magic
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = GroundPivotConstants.MOTION_CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = GroundPivotConstants.MOTION_ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = GroundPivotConstants.MOTION_JERK;
    pivotConfig.MotionMagic = motionMagicConfigs;

    // Configure Mechanism Reduction
    // This is the ratio between motor rotations and mechanism rotations
    // For example, if the motor needs to rotate 10 times to rotate the mechanism once,
    // the SensorToMechanismRatio would be 10.0
    // This allows us to use degrees directly as our control unit
    pivotConfig.Feedback.SensorToMechanismRatio = Constants.PhysicalConstants.groundPivotReduction;
    
    // Set neutral mode to brake
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    
    // Add voltage compensation
    pivotConfig.Voltage.PeakForwardVoltage = 12.0; // 12V compensation
    pivotConfig.Voltage.PeakReverseVoltage = -12.0;
    pivotConfig.Voltage.SupplyVoltageTimeConstant = 0.1;
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Apply Configuration
    PhoenixHelpers.tryConfig(() -> pivotMotor.getConfigurator().apply(pivotConfig));
    if (RobotBase.isReal()) {
      PhoenixHelpers.tryConfig(() -> pivotMotor.setPosition(197.0 / 360));
    }
  }

  @Override
  protected void periodicImpl() {
    // Handle zeroing first
    if (isZeroingPivot) {
      handlePivotZeroPeriodic();
      return;
    }

    // SmartDashboard.putNumber("AAAA", pivotMotor.signals().torqueCurrent().getValueAsDouble());

    // Convert degrees to rotations for motion magic
    // Since we've set the SensorToMechanismRatio, we need to convert our
    // desired angle in degrees to rotations of the mechanism
    double targetRotations = constrainedSetpoint / 360.0;
    
    pivotMotor.setControl(motionMagicRequest.withPosition(targetRotations).withSlot(0));
  }

  /**
   * Sets the ground pivot position using a predefined GroundPivot enum
   * 
   * @param position The GroundPivotPosition enum value
   */
  public void applySetpoint(GroundPivotPosition position) {
    applySetpoint(position.getDegrees());
  }

  /**
   * Gets the current angle of the ground pivot in degrees.
   * 
   * @return The current angle in degrees.
   */
  public double getPivotDegrees() {
    // Get the position in rotations and convert to degrees
    // The SensorToMechanismRatio is automatically applied by the Phoenix library
    return pivotMotor.signals().position().getValueAsDouble() * 360.0;
  }

  /**
   * Checks if the ground pivot is within a deadband of the desired setpoint
   * @return true if ground pivot is at setpoint
   */
  @Override
  public boolean atSetpoint() {
    return Math.abs(setpoint - getPivotDegrees()) < GroundPivotConstants.DEAD_ZONE;
  }

  /*             */
  /* Constraints */
  /*             */

  @Override
  protected void applyConstraints() {
    runConstraint(MechanismLimitsConstraint(), "Mechanism Limits");
  }

  public double MechanismLimitsConstraint() {
    return MathUtil.clamp(setpoint, GroundPivotConstants.MIN_ANGLE, GroundPivotConstants.MAX_ANGLE);
  }

  /*             */
  /*   Network   */
  /*             */

  /**
   * This method is called automatically by the SubsystemNetworkManager
   */
  @Override
  public void updateNetwork() {
    groundPivotAngleNT.set(getPivotDegrees());
    groundPivotisZeroingNT.set(isZeroingPivot);
    groundPivotAtSetpointNT.set(atSetpoint());
  }

  /*             */
  /*   Zeroing   */
  /*             */

  /**
   * Run periodically while zeroing pivot
   */
  private void handlePivotZeroPeriodic() {
    if (zeroingDebounceTrigger.getAsBoolean()) {
      pivotMotor.set(0);
      pivotMotor.setPosition(0);
      applySetpoint(0);
      
      isZeroingPivot = false;
      DriverStation.reportWarning("Ground Pivot zeroed successfully", false);
      
      return;
    }
    pivotMotor.set(GroundPivotConstants.ZEROING_SPEED);
  }

  /**
  * Begins zeroing the pivot.
  */
  public void zeroPivot() {
    if (isZeroingPivot) {
      isZeroingPivot = false;
      pivotMotor.set(0);
      DriverStation.reportWarning("Ground Pivot zeroing canceled", false);
          
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

  /*              */
  /*  Simulation  */
  /*              */

  @Override 
  public void simulationPeriodic() {
    var talonFXSim = pivotMotor.getSimState();

    var simResult = pivotSim.Evaluate(constrainedSetpoint / 360, 0.02);

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // WPILIB sim objects return mechanism position/velocity (after gear ratio)
    talonFXSim.setRawRotorPosition(simResult.get(0) * PhysicalConstants.groundPivotReduction);
    talonFXSim.setRotorVelocity(simResult.get(1) * PhysicalConstants.groundPivotReduction);
  }
}

