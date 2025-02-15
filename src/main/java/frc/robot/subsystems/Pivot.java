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
import frc.robot.data.Constants.ScoringConstants.ScoringLevel;
import frc.robot.subsystems.DynamicPathingSubsystem.DynamicPathingSituation;
import frc.robot.subsystems.Elevator.CollisionType;
import frc.robot.utils.NetworkUser;
import frc.robot.utils.SubsystemNetworkManager;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;

import static frc.robot.RobotContainer.intakeSubsystem;
import static frc.robot.RobotContainer.elevatorSubsystem;
import static frc.robot.data.Constants.ManipulatorConstants.*;

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
    private final DynamicMotionMagicVoltage motionMagicRequest = new DynamicMotionMagicVoltage(0, ManipulatorConstants.PIVOT_MOTION_CRUISE_VELOCITY, ManipulatorConstants.PIVOT_MOTION_ACCELERATION, ManipulatorConstants.PIVOT_MOTION_JERK);

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

        Slot1Configs slot1Configs = new Slot1Configs();
        slot1Configs.kP = Constants.ManipulatorConstants.PIVOT_kP_ALGEA_SLOW;
        slot1Configs.kI = Constants.ManipulatorConstants.PIVOT_kI;
        slot1Configs.kS = Constants.ManipulatorConstants.PIVOT_kS;
        slot1Configs.kD = Constants.ManipulatorConstants.PIVOT_kD;
        pivotConfigs.Slot1 = slot1Configs;

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
        // Real jank but ok
        int slot = 0;
        if (intakeSubsystem.isAlgaeLoaded()) {
            // while algea is loaded, use a slower profile
            // motionMagicRequest.Velocity = 0.00001; // rps
            // motionMagicRequest.Acceleration = ManipulatorConstants.PIVOT_MOTION_ACCELERATION / 25; // rot/s^2
            slot = 1;
        }

        // Update motor controls

        double chosenPivotAngle = 0;
        Elevator.CollisionType collisionPrediction = RobotContainer.elevatorSubsystem.getCurrentCollisionPotential();

        if (elevatorSubsystem.getElevatorPositionMeters() <= ElevatorConstants.COLLISION_ZONE_UPPER &&
            intakeSubsystem.isAlgaeLoaded() && pivotSetpointAngle < PivotPosition.CLEARANCE_POSITION_ALGEA.getDegrees()) {

            // if we have an algae, we can't fully retract when we are below the crossbar of the elevator
            chosenPivotAngle = PivotPosition.CLEARANCE_POSITION_ALGEA.getDegrees();
        }
        else if (collisionPrediction == CollisionType.NONE || pivotSetpointAngle > ElevatorConstants.MIN_ELEVATOR_PIVOT_ANGLE) {
            // Check for bumper collision, and limit angle if so
            if (isInBumperDangerZone() && pivotSetpointAngle > ManipulatorConstants.PIVOT_BUMPER_CLEARANCE_ANGLE) {
                // Move to max safe angle
                chosenPivotAngle = ManipulatorConstants.PIVOT_BUMPER_CLEARANCE_ANGLE;
            } else {
                // If we're past the safety angle, or aren't in danger of hitting anything, move pivot normally
                chosenPivotAngle = pivotSetpointAngle;
            }
        } else {
            // Not safe in some way, move pivot out of the way
            chosenPivotAngle = PivotPosition.CLEARANCE_POSITION.getDegrees();
        }

        // Auto dodge L4
        if (RobotContainer.dynamicPathingSubsystem.getCoralScoringLevel() == ScoringLevel.L4 && 
            RobotContainer.dynamicPathingSubsystem.getLastPathingSituation() == DynamicPathingSituation.REEF_CORAL &&
            !RobotContainer.isOperatorOverride) {
            if (pivotSetpointAngle > ManipulatorConstants.PIVOT_L4_CLEARANCE_ANGLE && isInL4DangerZone()) {
                chosenPivotAngle = ManipulatorConstants.PIVOT_L4_CLEARANCE_ANGLE;
                System.out.println("DANGER ZONE L4");
            }
        }

        if (RobotContainer.elevatorSubsystem.isZeroing()) {
            // Not safe since zeroing
            chosenPivotAngle = PivotPosition.CLEARANCE_POSITION.getDegrees();
        }

        pivot.setControl(motionMagicRequest.withPosition(chosenPivotAngle / 360).withSlot(slot));
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
        return RobotContainer.elevatorSubsystem.getElevatorPositionMeters() <= ElevatorConstants.PIVOT_BUMPER_CLEAR_HEIGHT ||
            RobotContainer.elevatorSubsystem.getElevatorSetpointMeters() <= ElevatorConstants.PIVOT_BUMPER_CLEAR_HEIGHT;
    }

    public boolean isInL4DangerZone() {
        return RobotContainer.elevatorSubsystem.getElevatorPositionMeters() >= ElevatorConstants.PIVOT_L4_CLEAR_HEIGHT_MIN &&
            RobotContainer.elevatorSubsystem.getElevatorPositionMeters() <= ElevatorConstants.PIVOT_L4_CLEAR_HEIGHT_MAX;
    }
}
