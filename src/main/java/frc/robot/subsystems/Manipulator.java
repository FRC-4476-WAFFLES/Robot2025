// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.data.Constants;
import frc.robot.utils.NetworkUser;
import frc.robot.utils.SubsystemNetworkManager;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.MathUtil;
import static frc.robot.data.Constants.ManipulatorConstants.*;

/**
 * The Manipulator subsystem handles the robot's intake and pivot mechanisms.
 * It controls:
 * - An intake motor for collecting game pieces
 * - A pivot motor for positioning the intake
 * - A CANCoder for absolute position feedback
 * - A LaserCan sensor for detecting game pieces
 */
public class Manipulator extends SubsystemBase implements NetworkUser {
    // Hardware Components
    private final TalonFX intake;
    private final TalonFX pivot;
    private final CANcoder pivotAbsoluteEncoder;
    private final LaserCan laserCan;

    // Control Objects
    private final DutyCycleOut intakeDutyCycle = new DutyCycleOut(0);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);


    // State variables
    public boolean algaeLoaded = false;
    private double intakeSpeed = 0;
    private double pivotSetpointAngle = 0;

    // Network Tables
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable pivotTable = inst.getTable("Pivot");
    private final DoublePublisher pivotSetpointNT = pivotTable.getDoubleTopic("Setpoint (Degrees)").publish();
    private final DoublePublisher pivotAngleNT = pivotTable.getDoubleTopic("Current Angle (Degrees)").publish();
    private final DoublePublisher laserCanDistanceNT = pivotTable.getDoubleTopic("LaserCan Distance (mm)").publish();


    // -------------------- Tuning Code --------------------
    // private NetworkConfiguredPID networkPIDConfiguration = new NetworkConfiguredPID(getName(), this::updatePID);
    
    // public void updatePID() {
    //   var slot0Configs = new Slot0Configs();
    //   slot0Configs.kS = networkPIDConfiguration.getS(); // Static feedforward
    //   slot0Configs.kP = networkPIDConfiguration.getP(); 
    //   slot0Configs.kI = networkPIDConfiguration.getI(); 
    //   slot0Configs.kD = networkPIDConfiguration.getD(); 

    //   pivot.getConfigurator().apply(slot0Configs);


    //   MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    //   motionMagicConfigs.MotionMagicCruiseVelocity = networkPIDConfiguration.getMotionMagicCruiseVelocity(); 
    //   motionMagicConfigs.MotionMagicAcceleration = networkPIDConfiguration.getMotionMagicAcceleration();
    //   motionMagicConfigs.MotionMagicJerk = networkPIDConfiguration.getMotionMagicJerk(); 

    //   pivot.getConfigurator().apply(motionMagicConfigs);
    // }

    public Manipulator() {
        SubsystemNetworkManager.RegisterNetworkUser(this);

        // Initialize hardware
        intake = new TalonFX(Constants.CANIds.intakeMotor);
        pivot = new TalonFX(Constants.CANIds.pivotMotor);
        pivotAbsoluteEncoder = new CANcoder(Constants.CANIds.pivotAbsoluteEncoder);
        
        // Configure hardware
        configureCANCoder();
        configurePivotMotor();
        configureIntakeMotor();

        // Initialize LaserCan with error handling
        try {
            laserCan = new LaserCan(Constants.CANIds.laserCan);
        } catch (Exception e) {
            throw new RuntimeException("Failed to initialize LaserCan: " + e.getMessage());
        }

        // Initialize position
        resetInternalEncoder();
    }

    /**
     * Configures the CANCoder with appropriate offset
     */
    private void configureCANCoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = Constants.PhysicalConstants.pivotAbsoluteEncoderOffset;
        pivotAbsoluteEncoder.getConfigurator().apply(config);
    }

    /**
     * Configures the pivot motor with motion magic and current limits
     */
    private void configurePivotMotor() {
        TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();

        // Current limits
        CurrentLimitsConfigs pivotCurrentLimit = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.ManipulatorConstants.STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true);
        pivotConfigs.CurrentLimits = pivotCurrentLimit;

        // Motion Magic
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.ManipulatorConstants.PIVOT_MOTION_CRUISE_VELOCITY)
            .withMotionMagicAcceleration(Constants.ManipulatorConstants.PIVOT_MOTION_ACCELERATION)
            .withMotionMagicJerk(Constants.ManipulatorConstants.PIVOT_MOTION_JERK);
        pivotConfigs.MotionMagic = motionMagicConfigs;

        // PID
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = Constants.ManipulatorConstants.PIVOT_kP;
        slot0Configs.kI = Constants.ManipulatorConstants.PIVOT_kI;
        slot0Configs.kD = Constants.ManipulatorConstants.PIVOT_kD;
        slot0Configs.kS = Constants.ManipulatorConstants.PIVOT_kS;
        pivotConfigs.Slot0 = slot0Configs;

        // For when CANCoder is not present
        pivotConfigs.Feedback.SensorToMechanismRatio = Constants.PhysicalConstants.pivotReduction;
        // For when CANCoder is present
        // pivotConfigs.Feedback.RotorToSensorRatio = Constants.PhysicalConstants.pivotReduction;
        // pivotConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // pivotConfigs.Feedback.FeedbackRemoteSensorID = pivotAbsoluteEncoder.getDeviceID();
        pivotConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        pivot.getConfigurator().apply(pivotConfigs);
    }

    /**
     * Configures the intake motor with current limits
     */
    private void configureIntakeMotor() {
        TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
        CurrentLimitsConfigs intakeCurrentLimit = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.ManipulatorConstants.STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true);
        intakeConfigs.CurrentLimits = intakeCurrentLimit;
        intake.getConfigurator().apply(intakeConfigs);
    }

    @Override
    public void periodic() {
        // Update motor controls
        //pivot.setControl(motionMagicRequest.withPosition(pivotSetpointAngle / 360).withSlot(0));
        intake.setControl(intakeDutyCycle.withOutput(intakeSpeed));
    }

    /**
     * Sets the target angle of the pivot mechanism
     * @param setpoint Target position in degrees or a PivotPosition enum
     * 
     */
    public void setPivotSetpoint(Object setpoint) {
        double targetDegrees;
        // Check if input is an enum value (e.g., PivotPosition.REST_POSITION)
        if (setpoint instanceof Constants.ManipulatorConstants.PivotPosition) {
            targetDegrees = ((Constants.ManipulatorConstants.PivotPosition) setpoint).getDegrees();
        } 
        // Check if input is a number (e.g., 45.0 or 45)
        else if (setpoint instanceof Double || setpoint instanceof Integer) {
            targetDegrees = ((Number) setpoint).doubleValue();
        } 
        // If neither enum nor number, throw error
        else {
            throw new IllegalArgumentException("Setpoint must be a PivotPosition enum or a number");
        }
        
        pivotSetpointAngle = MathUtil.clamp(targetDegrees, Constants.ManipulatorConstants.PIVOT_MIN_ANGLE, Constants.ManipulatorConstants.PIVOT_MAX_ANGLE);
    }

    /**
     * Gets the current pivot angle
     * @return Current angle in degrees
     */
    public double getPivotPosition() {
        return pivot.getPosition().getValueAsDouble() * 360;
    }

    /**
     * Sets the intake motor speed
     * @param speed Speed value between -1.0 and 1.0
     */
    public void setIntakeSpeed(double speed) {
        if (speed < -1.0 || speed > 1.0) {
            throw new IllegalArgumentException("Intake speed must be between -1.0 and 1.0");
        }
        this.intakeSpeed = speed;
    }

    /**
     * Checks if algae is present in the intake based on current draw
     * @return true if algae is detected
     */
    public void detectAlgaeLoaded() {
        if (intake.getStatorCurrent().getValueAsDouble() > Constants.ManipulatorConstants.ALGAE_CURRENT_THRESHOLD && isIntakingAlgae()) {
          algaeLoaded = true;
        }
        else if(isOuttakingAlgae()) {
          algaeLoaded = false;
        }
    }

    public boolean isAlgaeLoaded() {
        return algaeLoaded;
    }

    /**
     * Checks if coral is loaded using the laser distance sensor
     * @return true if coral is detected within threshold distance
     */
    public boolean isCoralLoaded() {
        var measurement = laserCan.getMeasurement();
        return measurement != null && 
               measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT &&
               measurement.distance_mm <= Constants.ManipulatorConstants.CORAL_LOADED_DISTANCE_THRESHOLD;
    }

    public boolean isIntakingCoral() {
        return !isCoralLoaded() && intake.getVelocity().getValueAsDouble() > 0;
    }

    public boolean isIntakingAlgae() {
        return !isAlgaeLoaded() && intake.getVelocity().getValueAsDouble() < 0;
    }

    public boolean isOuttakingAlgae() {
        return isAlgaeLoaded() && intake.getVelocity().getValueAsDouble() > 0;
    }

    public boolean isOuttakingCoral() {
        return isCoralLoaded() && intake.getVelocity().getValueAsDouble() < 0;
    }

    /**
     * Checks if pivot is at the target position
     * @return true if within deadband of setpoint
     */
    public boolean isPivotAtSetpoint() {
        return Math.abs(getPivotPosition() - pivotSetpointAngle) < Constants.ManipulatorConstants.PIVOT_ANGLE_DEADBAND;
    }

    private void resetInternalEncoder() {
        setPivotSetpoint(Constants.ManipulatorConstants.PivotPosition.REST_POSITION);
    }

    /**
     * This method is called automatically by the SubsystemNetworkManager
     */
    @Override
    public void updateNetwork() {
        pivotSetpointNT.set(pivotSetpointAngle);
        pivotAngleNT.set(getPivotPosition());
        laserCanDistanceNT.set(laserCan.getMeasurement().distance_mm);
    }

    @Override
    public void initializeNetwork() {
        // Network initialization if needed
    }
}
