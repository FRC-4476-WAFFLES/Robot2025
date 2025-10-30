// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.CANIds;
import frc.robot.data.Constants.ElevatorConstants;
import frc.robot.data.Constants.ManipulatorConstants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.subsystems.superstructure.Elevator.CollisionType;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.utils.PhoenixHelpers;
import frc.robot.utils.SecondOrderSim;
import frc.robot.utils.IO.CANcoderIO;
import frc.robot.utils.IO.TalonFXIO;
import frc.robot.utils.lib.WafflesMechanism;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import edu.wpi.first.math.util.Units;


/**
 * The Manipulator subsystem handles the robot's pivot mechanism.
 * It controls:
 * - A pivot motor for positioning the intake
 * - A CANCoder for absolute position feedback
 */
public class Pivot extends WafflesMechanism {
    // Hardware Components
    private final TalonFXIO pivot;
    // private final CANcoderIO pivotAbsoluteEncoder;

    private SecondOrderSim pivotSim;

    // Control Objects
    private final MotionMagicExpoVoltage motionMagicRequest = new MotionMagicExpoVoltage(0);

    // State variables
    private boolean isZeroingPivot = false;
    private boolean isThrowingAlgae = false;
    
    private MotionMagicConfigs motionMagicCoral;
    private MotionMagicConfigs motionMagicAlgae;
    private MotionMagicConfigs motionMagic;

    // Network Tables
    private final DoublePublisher pivotAngleNT = networkTable.getDoubleTopic("Current Angle (Degrees)").publish();
    private final DoublePublisher pivotVelocityNT = networkTable.getDoubleTopic("Current Velocity (rps)").publish();
    private final DoublePublisher pivotCurrentDrawNT = networkTable.getDoubleTopic("Current Draw (Amps)").publish();
    private final BooleanPublisher isZeroingNT = networkTable.getBooleanTopic("Is Zeroing").publish();
    protected final BooleanPublisher isAtSetpointNT = networkTable.getBooleanTopic("Pivot at Setpoint").publish();

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
        // Initialize hardware
        pivot = new TalonFXIO(CANIds.manipulatorPivot);
        // pivotAbsoluteEncoder = new CANcoderIO(CANIds.pivotAbsoluteEncoder);
        
        // Configure hardware
        configureCANCoder();
        configurePivotMotor();

        // Initialize position
        applySetpoint(SuperstructureState.ZERO);

        zeroingDebounceTrigger = new Trigger(() -> {
            return pivot.signals().torqueCurrent().getValueAsDouble() < -ManipulatorConstants.PIVOT_CURRENT_THRESHOLD;     
        }).debounce(ManipulatorConstants.ZERO_DEBOUNCE_TIME);

        if (RobotBase.isSimulation()) {
            pivotSim = new SecondOrderSim(2.5, 1, 0, 0);
        }
    }

    /**
     * Configures the CANCoder with appropriate offset
     */
    private void configureCANCoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = PhysicalConstants.pivotAbsoluteEncoderOffset;
        // PhoenixHelpers.tryConfig(() -> pivotAbsoluteEncoder.getConfigurator().apply(config));
    }

    /**
     * Configures the pivot motor with motion magic and current limits
     */
    @SuppressWarnings("unused")
    private void configurePivotMotor() {
        TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();

        // Current limits
        CurrentLimitsConfigs pivotCurrentLimit = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(ManipulatorConstants.STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true);
        pivotConfigs.CurrentLimits = pivotCurrentLimit;

        // Motion Magic
        motionMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(ManipulatorConstants.PIVOT_MOTION_CRUISE_VELOCITY)
            .withMotionMagicExpo_kV(ManipulatorConstants.PIVOT_SUPPLY_VOLTAGE / ManipulatorConstants.PIVOT_MOTION_CRUISE_VELOCITY)
            .withMotionMagicExpo_kA(ManipulatorConstants.PIVOT_SUPPLY_VOLTAGE / ManipulatorConstants.PIVOT_MOTION_ACCELERATION);
        pivotConfigs.MotionMagic = motionMagic;

        // Motion Magic Slow
        motionMagicCoral = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(ManipulatorConstants.PIVOT_MOTION_CRUISE_VELOCITY_CORAL)
            .withMotionMagicExpo_kV(ManipulatorConstants.PIVOT_SUPPLY_VOLTAGE / ManipulatorConstants.PIVOT_MOTION_CRUISE_VELOCITY_CORAL)
            .withMotionMagicExpo_kA(ManipulatorConstants.PIVOT_SUPPLY_VOLTAGE / ManipulatorConstants.PIVOT_MOTION_ACCELERATION_CORAL);
        
         motionMagicAlgae = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(ManipulatorConstants.PIVOT_MOTION_CRUISE_VELOCITY_CORAL)
            .withMotionMagicExpo_kV(ManipulatorConstants.PIVOT_SUPPLY_VOLTAGE / ManipulatorConstants.PIVOT_MOTION_CRUISE_VELOCITY_CORAL)
            .withMotionMagicExpo_kA(ManipulatorConstants.PIVOT_SUPPLY_VOLTAGE / ManipulatorConstants.PIVOT_MOTION_ACCELERATION_ALGAE);

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

        pivotConfigs.MotorOutput.DutyCycleNeutralDeadband = ManipulatorConstants.PIVOT_MOTOR_DEADBAND;

        
        if (PhysicalConstants.usePivotAbsoluteEncoder && RobotBase.isReal()) {
            // For when CANCoder is present
            pivotConfigs.Feedback.RotorToSensorRatio = PhysicalConstants.pivotReduction;
            pivotConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            // pivotConfigs.Feedback.FeedbackRemoteSensorID = pivotAbsoluteEncoder.getDeviceID();

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

        PhoenixHelpers.tryConfig(() -> pivot.getConfigurator().apply(pivotConfigs));
        if (RobotBase.isReal()) {
            PhoenixHelpers.tryConfig(() -> pivot.setPosition(3.0/360));
        }
    }

    public void initializeHooks() {
        // Speed switching
        RobotContainer.intakeSubsystem.manipulatorLoadedTrigger.onTrue(Commands.runOnce(() -> {
            if(RobotContainer.intakeSubsystem.isCoralLoaded()){
                pivot.getConfigurator().apply(motionMagicCoral);
            }else if(RobotContainer.intakeSubsystem.isAlgaeLoaded()){
                pivot.getConfigurator().apply(motionMagicAlgae);
            }
        }
        ));
        RobotContainer.intakeSubsystem.manipulatorLoadedTrigger.onFalse(Commands.runOnce(() -> pivot.getConfigurator().apply(motionMagic)));

    }

    @Override
    public void periodicImpl() {
        // Handle zeroing first
        if (isZeroingPivot) {
            handlePivotZeroPeriodic();
            return;
        }

        // Real jank but ok
        // int slot = 0;
        // if (RobotContainer.intakeSubsystem.isAlgaeLoaded()) {
        //     // while algae is loaded, use a slower profile
        //     slot = 1;
        // }

        // Account for zero not being vertical
        double pivotAngleFromVertical = getPivotPosition() - 40;
        double gravityFeedforward = ManipulatorConstants.PIVOT_kG_HORIZONTAL * Math.sin(Units.degreesToRadians(pivotAngleFromVertical));

        pivot.setControl(motionMagicRequest
            .withPosition(constrainedSetpoint / 360)
            .withFeedForward(gravityFeedforward)
            .withSlot(0)
        );
    }

    @Override
    protected void applyConstraints() {
        // Highest priority constraints should be run last
        runConstraint(frameCollisionConstraint(), "Frame Collision");
        runConstraint(firstStageCollisionConstraint(), "First Stage Collision");
        runConstraint(groundIntakeConstraint(), "Ground Intake");
        runConstraint(crossbarCollisionConstraint(), "Crossbar Collision");
        runConstraint(algaeConstraint(), "Algae Constraint");
        runConstraint(elevatorZeroingConstraint(),  "Elevator Zeroing");
        runConstraint(mechanismLimitsConstraint(), "Mechanism Limits");
    }

    /**
     * Sets the target angle of the pivot mechanism
     * @param setpoint A PivotPosition enum
     */
    public void applySetpoint(SuperstructureState setpoint) {
        applySetpoint(setpoint.getPivotAngle());
    }

    /**
     * Gets the current pivot angle
     * @return Current angle in degrees
     */
    public double getPivotPosition() {
        return pivot.signals().position().getValueAsDouble() * 360;
    }

    /**
     * Checks if pivot is at the target position
     * @return true if within deadband of setpoint
     */
    @Override
    public boolean atSetpoint() {
        double deadband = RobotContainer.intakeSubsystem.isAlgaeLoaded() ? ManipulatorConstants.PIVOT_ANGLE_DEADBAND * 3: ManipulatorConstants.PIVOT_ANGLE_DEADBAND;
        return Math.abs(getPivotPosition() - setpoint) < deadband;
    }

    /**
     * Sets if the pivot is throwing algae
     */
    public void setIsThrowingAlgae(boolean val) {
        isThrowingAlgae = val;
    }

    /*             */
    /* Constraints */
    /*             */

    private double elevatorZeroingConstraint() {
        return RobotContainer.superstructure.elevator.isZeroing() ? ManipulatorConstants.PIVOT_CLEARANCE_POSITION : constrainedSetpoint;
    }

    private double mechanismLimitsConstraint() {
        return MathUtil.clamp(constrainedSetpoint, ManipulatorConstants.PIVOT_MIN_ANGLE, ManipulatorConstants.PIVOT_MAX_ANGLE);
    }

    private double algaeConstraint() {
        if (
            RobotContainer.intakeSubsystem.isAlgaeLoaded() && 
            constrainedSetpoint < ManipulatorConstants.PIVOT_CLEARANCE_POSITION_ALGAE
        ) {
            // if we have an algae, we can't fully retract 
            return ManipulatorConstants.PIVOT_CLEARANCE_POSITION_ALGAE;
        }

        return constrainedSetpoint;
    }

    private double groundIntakeConstraint()
    {
        if (RobotContainer.groundSuperstructure.pivot.getPivotDegrees() < 20 ||
            RobotContainer.groundSuperstructure.pivot.getSetpoint() < 20) {
            // Ground intake is in
            if (constrainedSetpoint < ManipulatorConstants.PIVOT_CLEARANCE_POSITION) {
                if (RobotContainer.superstructure.elevator.getSetpoint() < 0.15 || 
                    RobotContainer.superstructure.elevator.getElevatorPositionMeters() < 0.15) {
                     return ManipulatorConstants.PIVOT_CLEARANCE_POSITION;
                }
            }
        }
        return constrainedSetpoint;
    }

    private double firstStageCollisionConstraint() {
        if (RobotContainer.superstructure.elevator.getSetpoint() < RobotContainer.superstructure.elevator.getElevatorPositionMeters() - 0.1) {
            // Elevator moving down
            if (RobotContainer.superstructure.elevator.getElevatorPositionMeters() > ElevatorConstants.FIRST_STAGE_START_HEIGHT) {
                if (constrainedSetpoint > ManipulatorConstants.FIRST_STAGE_AVOIDANCE_ANGLE) {
                    return ManipulatorConstants.FIRST_STAGE_AVOIDANCE_ANGLE;
                }
            }
        }
        return constrainedSetpoint;
    }

    private double crossbarCollisionConstraint() {
        CollisionType collisionPrediction = RobotContainer.superstructure.elevator.getCurrentCollisionPotential();
        
        if (collisionPrediction == CollisionType.NONE || constrainedSetpoint > ElevatorConstants.CROSSBAR_MIN_CLEAR_ANGLE) {
            // If we're past the safety angle, or aren't in danger of hitting anything, move pivot normally
            return constrainedSetpoint;
        }

        // Not safe in some way, move pivot out of the way
        return ManipulatorConstants.PIVOT_CLEARANCE_POSITION;
    }

    private double frameCollisionConstraint() {
        // Check for frame collision, and limit angle if needed
        if (isInFrameDangerZone() && RobotContainer.intakeSubsystem.isCoralLoaded()) {
            // (constrainedSetpoint, ManipulatorConstants.PIVOT_FRAME_MIN_CLEARANCE_ANGLE, ManipulatorConstants.PIVOT_FRAME_MAX_CLEARANCE_ANGLE);
            if (
                constrainedSetpoint < ManipulatorConstants.PIVOT_FRAME_UPPER_CLEARANCE_ANGLE &&
                constrainedSetpoint > ManipulatorConstants.PIVOT_FRAME_LOWER_CLEARANCE_ANGLE
            ) {
                // Setpoint is in danger zone
                double distanceToMin = Math.abs(constrainedSetpoint - ManipulatorConstants.PIVOT_FRAME_LOWER_CLEARANCE_ANGLE);
                double distanceToMax = Math.abs(constrainedSetpoint - ManipulatorConstants.PIVOT_FRAME_UPPER_CLEARANCE_ANGLE);

                if (distanceToMin < distanceToMax) {
                    return ManipulatorConstants.PIVOT_FRAME_LOWER_CLEARANCE_ANGLE;
                }
                return ManipulatorConstants.PIVOT_FRAME_UPPER_CLEARANCE_ANGLE;
            }
        }

        return constrainedSetpoint;
    }

    public boolean isInFrameDangerZone() {
        return RobotContainer.superstructure.elevator.getElevatorPositionMeters() <= ElevatorConstants.PIVOT_BUMPER_CLEAR_HEIGHT ||
            RobotContainer.superstructure.elevator.getSetpoint() <= ElevatorConstants.PIVOT_BUMPER_CLEAR_HEIGHT;
    }

    public boolean pastReefHitAngle() {
        return getPivotPosition() > ManipulatorConstants.PIVOT_REEF_CLEAR_ANGLE;
    }

    /*             */
    /*   Network   */
    /*             */

    @Override
    public void updateNetwork() {
        pivotAngleNT.set(getPivotPosition());
        isZeroingNT.set(isZeroingPivot);
        pivotCurrentDrawNT.set(pivot.signals().torqueCurrent().getValueAsDouble());
        pivotVelocityNT.set(pivot.signals().velocity().getValueAsDouble());
        isAtSetpointNT.set(atSetpoint());
    }

    /*             */
    /*   Zeroing   */
    /*             */

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
            applySetpoint(0);
            
            isZeroingPivot = false;
            DriverStation.reportWarning("Pivot zeroed successfully", false);
            
            return;
        }
        pivot.set(ManipulatorConstants.ZEROING_SPEED);
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

    /*              */
    /*  Simulation  */
    /*              */

    @Override 
    public void simulationPeriodic() {
        var talonFXSim = pivot.getSimState();

        var simResult = pivotSim.Evaluate(constrainedSetpoint / 360, 0.02);

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // WPILIB sim objects return mechanism position/velocity (after gear ratio)
        talonFXSim.setRawRotorPosition(simResult.get(0) * PhysicalConstants.pivotReduction);
        talonFXSim.setRotorVelocity(simResult.get(1) * PhysicalConstants.pivotReduction);
    }
}
