// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.RobotContainer.elevatorSubsystem;
import static frc.robot.RobotContainer.intakeSubsystem;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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
import frc.robot.RobotContainer;
import frc.robot.data.Constants.CANIds;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.ElevatorConstants;
import frc.robot.data.Constants.ManipulatorConstants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.data.Constants.ManipulatorConstants.PivotPosition;
import frc.robot.subsystems.Elevator.CollisionType;
import frc.robot.utils.NetworkUser;
import frc.robot.utils.SubsystemNetworkManager;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import edu.wpi.first.math.util.Units;



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
    private final MotionMagicExpoVoltage motionMagicRequest = new MotionMagicExpoVoltage(0);
    // private final DynamicMotionMagicVoltage slowMotionMagicRequest = new DynamicMotionMagicVoltage(0, ManipulatorConstants.PIVOT_MOTION_CRUISE_VELOCITY / 25, ManipulatorConstants.PIVOT_MOTION_ACCELERATION / 25, ManipulatorConstants.PIVOT_MOTION_JERK / 25);

    // State variables
    private double pivotSetpointAngle = 0;
    private boolean isZeroingPivot = false;
    private boolean isThrowingAlgae = false;



    // Network Tables
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable pivotTable = inst.getTable("Pivot");
    private final DoublePublisher pivotSetpointNT = pivotTable.getDoubleTopic("Setpoint (Degrees)").publish();
    private final DoublePublisher realPivotSetpointNT = pivotTable.getDoubleTopic("Real Setpoint (Degrees)").publish();
    private final DoublePublisher pivotAngleNT = pivotTable.getDoubleTopic("Current Angle (Degrees)").publish();
    private final DoublePublisher pivotVelocityNT = pivotTable.getDoubleTopic("Current Velocity (rps)").publish();
    private final DoublePublisher pivotCurrentDrawNT = pivotTable.getDoubleTopic("Current Draw (Amps)").publish();
    private final BooleanPublisher isAtSetpointNT = pivotTable.getBooleanTopic("Pivot at Setpoint").publish();
    private final BooleanPublisher isZeroingNT = pivotTable.getBooleanTopic("Is Zeroing").publish();

    private Trigger zeroingDebounceTrigger;

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
        SubsystemNetworkManager.RegisterNetworkUser(this, true, CodeConstants.SUBSYSTEM_NT_UPDATE_RATE);

        // Initialize hardware
        pivot = new TalonFX(CANIds.pivotMotor);
        pivotAbsoluteEncoder = new CANcoder(CANIds.pivotAbsoluteEncoder);
        
        // Configure hardware
        configureCANCoder();
        configurePivotMotor();

        // Initialize position
        resetInternalEncoder();

        zeroingDebounceTrigger = new Trigger(() -> {
            return pivot.getTorqueCurrent().getValueAsDouble() < -ManipulatorConstants.PIVOT_CURRENT_THRESHOLD;     
        }).debounce(ManipulatorConstants.ZERO_DEBOUNCE_TIME);
    }

    /**
     * Configures the CANCoder with appropriate offset
     */
    private void configureCANCoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = PhysicalConstants.pivotAbsoluteEncoderOffset;
        pivotAbsoluteEncoder.getConfigurator().apply(config);
    }

    /**
     * Configures the pivot motor with motion magic and current limits
     */
    private void configurePivotMotor() {
        TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();

        // Current limits
        CurrentLimitsConfigs pivotCurrentLimit = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(ManipulatorConstants.STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true);
        pivotConfigs.CurrentLimits = pivotCurrentLimit;

        // Motion Magic
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
            // .withMotionMagicCruiseVelocity(Constants.ManipulatorConstants.PIVOT_MOTION_CRUISE_VELOCITY)
            // .withMotionMagicAcceleration(Constants.ManipulatorConstants.PIVOT_MOTION_ACCELERATION)
            // .withMotionMagicJerk(Constants.ManipulatorConstants.PIVOT_MOTION_JERK)
            .withMotionMagicCruiseVelocity(ManipulatorConstants.PIVOT_MOTION_CRUISE_VELOCITY)
            .withMotionMagicExpo_kV(ManipulatorConstants.PIVOT_SUPPLY_VOLTAGE / ManipulatorConstants.PIVOT_MOTION_CRUISE_VELOCITY)
            .withMotionMagicExpo_kA(ManipulatorConstants.PIVOT_SUPPLY_VOLTAGE / ManipulatorConstants.PIVOT_MOTION_ACCELERATION);
        pivotConfigs.MotionMagic = motionMagicConfigs;

        // PID
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = ManipulatorConstants.PIVOT_kP;
        slot0Configs.kI = ManipulatorConstants.PIVOT_kI;
        slot0Configs.kD = ManipulatorConstants.PIVOT_kD;
        slot0Configs.kS = ManipulatorConstants.PIVOT_kS;
        pivotConfigs.Slot0 = slot0Configs;

        Slot1Configs slot1Configs = new Slot1Configs();
        slot1Configs.kP = ManipulatorConstants.PIVOT_kP_ALGAE_SLOW;
        slot1Configs.kI = ManipulatorConstants.PIVOT_kI;
        slot1Configs.kD = ManipulatorConstants.PIVOT_kD;
        slot1Configs.kS = ManipulatorConstants.PIVOT_kS;
        pivotConfigs.Slot1 = slot1Configs;

        Slot2Configs slot2Configs = new Slot2Configs();
        slot2Configs.kI = ManipulatorConstants.PIVOT_kI;
        slot2Configs.kP = ManipulatorConstants.PIVOT_kP;
        slot2Configs.kS = ManipulatorConstants.PIVOT_kS;
        slot2Configs.kD = ManipulatorConstants.PIVOT_kD;
        
        pivotConfigs.Slot2 = slot2Configs;

        pivotConfigs.MotorOutput.DutyCycleNeutralDeadband = ManipulatorConstants.PIVOT_MOTOR_DEADBAND;

        
        if (PhysicalConstants.usePivotAbsoluteEncoder) {
            // For when CANCoder is present
            pivotConfigs.Feedback.RotorToSensorRatio = PhysicalConstants.pivotReduction;
            pivotConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            pivotConfigs.Feedback.FeedbackRemoteSensorID = pivotAbsoluteEncoder.getDeviceID();

            pivotConfigs.Feedback.SensorToMechanismRatio = 1;
        } else { 
            // For when CANCoder is not present
            pivotConfigs.Feedback.SensorToMechanismRatio = PhysicalConstants.pivotReduction;

            pivotConfigs.Feedback.RotorToSensorRatio = 1;
        }
        

        pivotConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Add voltage compensation
        pivotConfigs.Voltage.PeakForwardVoltage = ManipulatorConstants.PIVOT_SUPPLY_VOLTAGE; // 12V compensation
        pivotConfigs.Voltage.PeakReverseVoltage = -ManipulatorConstants.PIVOT_SUPPLY_VOLTAGE;
        pivotConfigs.Voltage.SupplyVoltageTimeConstant = 0.1;
        pivotConfigs.CurrentLimits.StatorCurrentLimit = 60;

        pivot.getConfigurator().apply(pivotConfigs);
    }

    @Override
    public void periodic() {
        // Handle zeroing first
        if (isZeroingPivot) {
            handlePivotZeroPeriodic();
            return;
        }

        // Real jank but ok
        int slot = 0;
        if (intakeSubsystem.isAlgaeLoaded() && !isThrowingAlgae) {
            // while algae is loaded, use a slower profile
            slot = 1;
        }

        // Update motor controls

        double chosenPivotAngle = 0;
        Elevator.CollisionType collisionPrediction = RobotContainer.elevatorSubsystem.getCurrentCollisionPotential();

        if (isInAlgaeDangerZone() &&
            intakeSubsystem.isAlgaeLoaded() && 
            pivotSetpointAngle < PivotPosition.CLEARANCE_POSITION_ALGAE.getDegrees()
        ) {

            // if we have an algae, we can't fully retract when we are below the crossbar of the elevator
            chosenPivotAngle = PivotPosition.CLEARANCE_POSITION_ALGAE.getDegrees();
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


        // Don't think we need this anymore since we are far away when autopathing for L4. Will help with stability of the pivot when scoring L4.
        // // Auto dodge L4
        // if (RobotContainer.dynamicPathingSubsystem.getCoralScoringLevel() == ScoringLevel.L4 && 
        //     RobotContainer.dynamicPathingSubsystem.getCurrentPathingSituation() == DynamicPathingSituation.REEF_CORAL &&
        //     !RobotContainer.isOperatorOverride) {
        //     if (pivotSetpointAngle > ManipulatorConstants.PIVOT_L4_CLEARANCE_ANGLE && isInL4DangerZone()) {
        //         chosenPivotAngle = ManipulatorConstants.PIVOT_L4_CLEARANCE_ANGLE;
        //         // System.out.println("DANGER ZONE L4");
        //     }
        // }

        if (RobotContainer.elevatorSubsystem.isZeroing()) {
            // Not safe since zeroing
            chosenPivotAngle = PivotPosition.CLEARANCE_POSITION.getDegrees();
        }

        realPivotSetpointNT.set(chosenPivotAngle);

        // Account for zero not being vertical
        double pivotAngleFromVertical = getPivotPosition() - 40;
        double gravityFeedforward = ManipulatorConstants.PIVOT_kG_HORIZONTAL * Math.sin(Units.degreesToRadians(pivotAngleFromVertical));

        pivot.setControl(motionMagicRequest
            .withPosition(chosenPivotAngle / 360)
            .withFeedForward(gravityFeedforward)
            .withSlot(slot)
        );
    }

    /**
     * Run periodically while zeroing pivot
     */
    private void handlePivotZeroPeriodic() {
        if (PhysicalConstants.usePivotAbsoluteEncoder) {
            isZeroingPivot = false;
            return;
        }

        if (zeroingDebounceTrigger.getAsBoolean()) {

            pivot.set(0);
            pivot.setPosition(0.0);
            setPivotPosition(0);
            
            isZeroingPivot = false;
            DriverStation.reportWarning("Pivot zeroed successfully", false);
            
            return;
        }
        pivot.set(ManipulatorConstants.ZEROING_SPEED);
    }

    /**
     * Sets the target angle of the pivot mechanism
     * @param setpoint A PivotPosition enum
     */
    public void setPivotPosition(PivotPosition setpoint) {
        setPivotPosition(setpoint.getDegrees());
    }

    /**
     * Sets the target angle of the pivot mechanism
     * @param setpoint A number in degrees
     */
    public void setPivotPosition(double targetDegrees) {
        pivotSetpointAngle = MathUtil.clamp(targetDegrees, ManipulatorConstants.PIVOT_MIN_ANGLE, ManipulatorConstants.PIVOT_MAX_ANGLE);
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
        double deadband = RobotContainer.intakeSubsystem.isAlgaeLoaded() ? ManipulatorConstants.PIVOT_ANGLE_DEADBAND * 3: ManipulatorConstants.PIVOT_ANGLE_DEADBAND;
        return Math.abs(getPivotPosition() - pivotSetpointAngle) < deadband;
    }

    /*
     * Zero pivot encoder
     */
    private void resetInternalEncoder() {
        setPivotPosition(PivotPosition.ZERO);
    }

    /**
     * This method is called automatically by the SubsystemNetworkManager
     */
    @Override
    public void updateNetwork() {
        isAtSetpointNT.set(isPivotAtSetpoint());
        pivotSetpointNT.set(pivotSetpointAngle);
        pivotAngleNT.set(getPivotPosition());
        isZeroingNT.set(isZeroingPivot);
        pivotCurrentDrawNT.set(pivot.getTorqueCurrent().getValueAsDouble());
        pivotVelocityNT.set(pivot.getVelocity().getValueAsDouble());
    }

    @Override
    public void initializeNetwork() {
        // Network initialization if needed
    }

    /*
     * Returns if the elevator is (or will be) in a situation where the pivot can hit the bumpers
     */
    public boolean isInBumperDangerZone() {
        return RobotContainer.elevatorSubsystem.getElevatorPositionMeters() <= ElevatorConstants.PIVOT_BUMPER_CLEAR_HEIGHT ||
            RobotContainer.elevatorSubsystem.getElevatorSetpointMeters() <= ElevatorConstants.PIVOT_BUMPER_CLEAR_HEIGHT;
    }

    /*
     * Returns if the elevator is (or will be) in a situation where the pivot can hit the L4 posts
     */
    public boolean isInL4DangerZone() {
        return RobotContainer.elevatorSubsystem.getElevatorPositionMeters() >= ElevatorConstants.PIVOT_L4_CLEAR_HEIGHT_MIN &&
            RobotContainer.elevatorSubsystem.getElevatorPositionMeters() <= ElevatorConstants.PIVOT_L4_CLEAR_HEIGHT_MAX;
    }

    /*
     * Returns if the elevator is (or will be) in a situation where the pivot with algae can hit the robot's structure
     */
    public boolean isInAlgaeDangerZone() {
        return elevatorSubsystem.getElevatorPositionMeters() <= ElevatorConstants.COLLISION_ZONE_UPPER ||
        elevatorSubsystem.getElevatorSetpointMeters() <= ElevatorConstants.COLLISION_ZONE_UPPER;
    }

    /**
     * Begins zeroing the pivot.
     */
    public void zeroPivot() {
        if (PhysicalConstants.usePivotAbsoluteEncoder) {
            return;
        }

        if (isZeroingPivot) {
            isZeroingPivot = false;

            pivot.set(0);
            DriverStation.reportWarning("Pivot zeroing canceled", false);
            
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

    
    public void setIsThrowingAlgae(boolean val) {
        isThrowingAlgae = val;
    }
}
