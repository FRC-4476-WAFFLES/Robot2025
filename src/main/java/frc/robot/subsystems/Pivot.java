// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.data.Constants;
import frc.robot.data.Constants.ElevatorConstants;
import frc.robot.data.Constants.ManipulatorConstants;
import frc.robot.data.Constants.ManipulatorConstants.PivotPosition;
import frc.robot.subsystems.Elevator.CollisionType;
import frc.robot.utils.NetworkConfiguredPID;
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

import javax.print.event.PrintJobAttributeEvent;

/**
 * The Manipulator subsystem handles the robot's pivot mechanism.
 * It controls:
 * - A pivot motor for positioning the intake
 * - A CANCoder for absolute position feedback
 */
public class Pivot extends SubsystemBase implements NetworkUser {
    // Hardware Components
    private final TalonFX pivot;
    private final CANcoder pivotAbsoluteEncoder;

    // Control Objects
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    // State variables
    private double pivotSetpointAngle = 0;

    // Network Tables
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable pivotTable = inst.getTable("Pivot");
    private final DoublePublisher pivotSetpointNT = pivotTable.getDoubleTopic("Setpoint (Degrees)").publish();
    private final DoublePublisher pivotAngleNT = pivotTable.getDoubleTopic("Current Angle (Degrees)").publish();
    private final BooleanPublisher isAtSetpointNT = pivotTable.getBooleanTopic("Pivot at Setpoint").publish();


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

    //   System.out.println("Refreshing PID values from networktables for manipulator");
    // }

    public Pivot() {
        SubsystemNetworkManager.RegisterNetworkUser(this);

        // Initialize hardware
        pivot = new TalonFX(Constants.CANIds.pivotMotor);
        pivotAbsoluteEncoder = new CANcoder(Constants.CANIds.pivotAbsoluteEncoder);
        
        // Configure hardware
        configureCANCoder();
        configurePivotMotor();

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

        pivotConfigs.MotorOutput.DutyCycleNeutralDeadband = PIVOT_MOTOR_DEADBAND;

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

    @Override
    public void periodic() {
        // Update motor controls
        Elevator.CollisionType collisionPrediction = RobotContainer.elevatorSubsystem.getCurrentCollisionPotential();
        if (RobotContainer.elevatorSubsystem.isZeroing()) {
            // Not safe since zeroing
            pivot.setControl(motionMagicRequest.withPosition(PivotPosition.CLEARANCE_POSITION.getDegrees() / 360).withSlot(0));
        }

        if (collisionPrediction == CollisionType.NONE || pivotSetpointAngle > ElevatorConstants.MIN_ELEVATOR_PIVOT_ANGLE) {
            
            // Check for bumper collision, and limit angle if so
            if (isInBumperDangerZone() && pivotSetpointAngle > ManipulatorConstants.PIVOT_BUMPER_CLEARANCE_ANGLE) {
                // Move to max safe angle
                pivot.setControl(motionMagicRequest.withPosition(ManipulatorConstants.PIVOT_BUMPER_CLEARANCE_ANGLE / 360).withSlot(0));
            } else {
                // If we're past the safety angle, or aren't in danger of hitting anything, move pivot normally
                pivot.setControl(motionMagicRequest.withPosition(pivotSetpointAngle / 360).withSlot(0));
            }
        } else {
            // Not safe in some way, move pivot out of the way
            pivot.setControl(motionMagicRequest.withPosition(PivotPosition.CLEARANCE_POSITION.getDegrees() / 360).withSlot(0));
            // System.out.println(collisionPrediction);
        }
    }

    /**
     * Sets the target angle of the pivot mechanism
     * @param setpoint Target position in degrees or a PivotPosition enum
     * 
     */
    public void setPivotSetpoint(Object setpoint) {
        double targetDegrees;
        // Check if input is an enum value (e.g., PivotPosition.REST_POSITION)
        if (setpoint instanceof PivotPosition) {
            targetDegrees = ((PivotPosition) setpoint).getDegrees();
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
     * Gets the current pivot setpoint
     * @return Current setpoint angle in degrees
     */
    public double getPivotSetpoint() {
        return pivotSetpointAngle;
    }

    /**
     * Checks if pivot is at the target position
     * @return true if within deadband of setpoint
     */
    public boolean isPivotAtSetpoint() {
        return Math.abs(getPivotPosition() - pivotSetpointAngle) < Constants.ManipulatorConstants.PIVOT_ANGLE_DEADBAND;
    }

    private void resetInternalEncoder() {
        setPivotSetpoint(Constants.ManipulatorConstants.PivotPosition.CLEARANCE_POSITION);
    }

    /**
     * This method is called automatically by the SubsystemNetworkManager
     */
    @Override
    public void updateNetwork() {
        isAtSetpointNT.set(isPivotAtSetpoint());
        pivotSetpointNT.set(pivotSetpointAngle);
        pivotAngleNT.set(getPivotPosition());
        // laserCanDistanceNT.set(laserCANDistance);
        // coralLoadedNT.set(isCoralLoaded());
    }

    @Override
    public void initializeNetwork() {
        // Network initialization if needed
    }

    public boolean isInBumperDangerZone() {
        return RobotContainer.elevatorSubsystem.getElevatorPositionMeters() <= ElevatorConstants.PIVOT_BUMPER_CLEAR_HEIGHT;
    }
}
