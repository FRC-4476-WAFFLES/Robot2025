package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.data.Constants;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.ManipulatorConstants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.utils.NetworkUser;
import frc.robot.utils.SubsystemNetworkManager;

/**
 * The Intake subsystem handles the robot's intake mechanism.
 * It controls:
 * - An intake motor for collecting game pieces
 * - A LaserCan sensor for detecting game pieces
 */
public class Intake extends SubsystemBase implements NetworkUser{
    // Hardware Components
    private final TalonFX intake;
    private LaserCan intakeLaserCan;
    private LaserCan funnelLaserCan;

    // Control Objects
    private final MotionMagicVelocityVoltage intakeControlRequest = new MotionMagicVelocityVoltage(0);
    private final VoltageOut intakePositionRequest = new VoltageOut(0).withEnableFOC(true);
    private final PositionVoltage intakePositionControlRequest = new PositionVoltage(0).withSlot(1);

    // State Variables
    private double intakeLaserDistance = 0;
    private double funnelLaserDistance = 0;
    private double intakeSpeed = 0;
    private double targetPosition = 0;
    private boolean usePositionControl = false;
    private boolean noAlgaeFlag = false;
    private boolean algaeLoaded = false;

    private Timer algaeLossTimer = new Timer();

    private Trigger algaeDetectionTrigger;

    // Network Tables
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable intakeTable = inst.getTable("Intake");
    private final DoublePublisher intakeLaserCanDistanceNT = intakeTable.getDoubleTopic("Intake Laser Distance (mm)").publish();
    private final DoublePublisher funnelLaserCanDistanceNT = intakeTable.getDoubleTopic("Funnel Laser Distance (mm)").publish();
    private final BooleanPublisher coralLoadedNT = intakeTable.getBooleanTopic("Coral Loaded").publish();
    private final BooleanPublisher algaeLoadedNT = intakeTable.getBooleanTopic("Algae Loaded").publish();
    private final DoublePublisher intakeSetpointNT = intakeTable.getDoubleTopic("Intake Setpoint").publish();
    private final DoublePublisher intakeCurrentDrawNT = intakeTable.getDoubleTopic("Intake Current Draw").publish();
    private final DoublePublisher intakeVelocityNT = intakeTable.getDoubleTopic("Intake Velocity").publish();
    private final DoublePublisher intakePositionNT = intakeTable.getDoubleTopic("Intake Position").publish();
    private final DoublePublisher intakeTargetPositionNT = intakeTable.getDoubleTopic("Intake Target Position").publish();

    private final BooleanPublisher isIntakingAlgaeNT = intakeTable.getBooleanTopic("IsIntaking").publish();
    private final BooleanPublisher isOutakingAlgaeNT = intakeTable.getBooleanTopic("IsOutaking").publish();
    private final BooleanPublisher isPositionControlNT = intakeTable.getBooleanTopic("IsPositionControl").publish();

    public Intake() {
        SubsystemNetworkManager.RegisterNetworkUser(this, true, CodeConstants.SUBSYSTEM_NT_UPDATE_RATE);

        intake = new TalonFX(Constants.CANIds.intakeMotor);

        // Configure hardware
        configureIntakeMotor();
        configureLaserCAN();

        algaeLossTimer.reset();

        algaeDetectionTrigger = new Trigger(
            () -> intake.getStatorCurrent().getValueAsDouble() > ManipulatorConstants.ALGAE_CURRENT_THRESHOLD 
            && isIntakingAlgae() 
            && !isCoralLoaded()
        ).debounce(ManipulatorConstants.ALGAE_DETECTION_DEBOUNCE_TIME);
    }

    /**
     * Configures the laserCAN
     */
    private void configureLaserCAN() {
        // Initialize LaserCan with error handling
        try {
            intakeLaserCan = new LaserCan(Constants.CANIds.intakeLaserCan);
            intakeLaserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            intakeLaserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);

            funnelLaserCan = new LaserCan(Constants.CANIds.funnelLaserCan);
            intakeLaserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            intakeLaserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        } catch (Exception e) {
            // throw new RuntimeException("Failed to initialize LaserCan: " + e.getMessage());
            System.out.println("Failed to initialize LaserCan: " + e.getMessage());
            intakeLaserCan = null;
            funnelLaserCan = null;
        }
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

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.3;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kV = 0.33;
        slot0Configs.kG = 0.0;

        var slot1Configs = new Slot1Configs();
        slot1Configs.kP = 10.0; // Higher P gain for position control
        slot1Configs.kI = 0;
        slot1Configs.kD = 0.01;
        slot1Configs.kV = 0.2;
        slot1Configs.kG = 0.0;

        intakeConfigs.Slot0 = slot0Configs;
        intakeConfigs.Slot1 = slot1Configs;

        // Motion Magic
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration = 200;
        motionMagicConfigs.MotionMagicJerk = 0;
        intakeConfigs.MotionMagic = motionMagicConfigs;

        intakeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        intakeConfigs.Feedback.SensorToMechanismRatio = PhysicalConstants.intakeReduction;

        intakeConfigs.MotorOutput.DutyCycleNeutralDeadband = 0.01;

        intake.getConfigurator().apply(intakeConfigs);
    }
    
    @Override
    public void periodic() {
        if (algaeLossTimer.get() > 1) {
            algaeLoaded = false;
            algaeLossTimer.stop();
            algaeLossTimer.reset();

            System.out.print("===============\nLost Algae Detected.");
        }

        if (!usePositionControl) {
            if (Math.abs(intakeSpeed) < 0.01 && isAlgaeLoaded()) {
                // When algae is loaded, run intake slowly inward
                intake.setControl(intakeControlRequest.withVelocity(Constants.ManipulatorConstants.ALGAE_HOLD_SPEED).withSlot(0));

                if (intake.getStatorCurrent().getValueAsDouble() < 4) {
                    intake.setControl(intakeControlRequest.withVelocity(-120).withSlot(0));
                    
                    if (!algaeLossTimer.isRunning()) {
                        algaeLossTimer.reset();
                        algaeLossTimer.start();
                    }
                }
            } else if (Math.abs(intakeSpeed) < 0.01 && isCoralLoaded()) {
                intake.setControl(intakePositionRequest.withOutput(0));
            } else {
                intake.setControl(intakeControlRequest.withVelocity(intakeSpeed).withSlot(0));
            }
        } else {
            // Use position control
            intake.setControl(intakePositionControlRequest.withPosition(targetPosition));

            // Auto disable position control once at setpoint & not moving
            if (isAtTargetPosition() && isIntakeStopped()) {
                usePositionControl = false;
            }
        }

        // Update gamepeice sensing
        detectAlgaeLoaded();
        updateCoralSensors();
    }

    /**
     * Sets the intake motor speed
     * @param speed Speed value (rotations/s)
     */
    public void setIntakeSpeed(double speed) {
        this.intakeSpeed = speed;
    }

    /**
     * Temporarily prevents the intake from registering coral loads
     * @param val the value to set the flag to
     */
    public void setNoAlgaeFlag(boolean val) {
        noAlgaeFlag = val;
    }

    /**
     * Checks if algae is present in the intake based on current draw
     */
    private void detectAlgaeLoaded() {
        if (algaeDetectionTrigger.getAsBoolean()) {
            algaeLoaded = true;

            // Reset algae loss conditions
            algaeLossTimer.stop();
            algaeLossTimer.reset();
        } else if (isOuttakingAlgae()) {
            algaeLoaded = false;
        }
    }

    /**
     * Updates the coral sensor's internal state
     */
    private void updateCoralSensors() {
        if (intakeLaserCan != null) {
            var measurement = intakeLaserCan.getMeasurement();
            if (measurement != null) {
                if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                    intakeLaserDistance = measurement.distance_mm;
                }
            }
        }

        if (funnelLaserCan != null) {
            var measurement = funnelLaserCan.getMeasurement();
            if (measurement != null) {
                if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                    funnelLaserDistance = measurement.distance_mm;
                }
            }
        }
    }

    /**
     * Is algae loaded in the manipulator
     * @return a boolean
     */
    public boolean isAlgaeLoaded() {
        return algaeLoaded;
    }

    /**
     * Checks if coral is loaded using the laser distance sensor
     * @return true if coral is detected within threshold distance
     */
    public boolean isCoralLoaded() {
        return intakeLaserDistance <= Constants.ManipulatorConstants.CORAL_LOADED_DISTANCE_THRESHOLD;
    }

    /**
     * Checks if coral is in the funnel using the funne; laser distance sensor
     * @return true if coral is detected within threshold distance
     */
    public boolean funnelSeesCoral() {
        return funnelLaserDistance <= Constants.ManipulatorConstants.CORAL_LOADED_DISTANCE_THRESHOLD;
    }

    /* Helper methods for determining the intake's basic state */

    public boolean isIntakingAlgae() {
        return !isAlgaeLoaded() && intakeSpeed > 10 && !noAlgaeFlag;
    }

    public boolean isOuttakingAlgae() {
        return isAlgaeLoaded() && intake.getVelocity().getValueAsDouble() < -12;
    }

    public boolean isAtTargetPosition() {
        return Math.abs(intake.getPosition().getValueAsDouble() - targetPosition) < 0.1;
    }

    public boolean isIntakeStopped() {
        return Math.abs(intake.getVelocity().getValueAsDouble()) < 0.1;
    }

    /**
     * This method is called automatically by the SubsystemNetworkManager
     */
    @Override
    public void updateNetwork() {
        intakeLaserCanDistanceNT.set(intakeLaserDistance);
        funnelLaserCanDistanceNT.set(funnelLaserDistance);
        coralLoadedNT.set(isCoralLoaded());
        algaeLoadedNT.set(isAlgaeLoaded());
        intakeSetpointNT.set(intakeSpeed);
        intakeCurrentDrawNT.set(intake.getStatorCurrent().getValueAsDouble());
        intakeVelocityNT.set(intake.getVelocity().getValueAsDouble());
        intakePositionNT.set(intake.getPosition().getValueAsDouble());
        intakeTargetPositionNT.set(targetPosition);

        isIntakingAlgaeNT.set(isIntakingAlgae());
        isOutakingAlgaeNT.set(isOuttakingAlgae());
        isPositionControlNT.set(usePositionControl);
    }

    @Override
    public void initializeNetwork() {
        // Network initialization if needed
    }

    /**
     * Sets the target position for the intake motor and enables position control
     * @param position The target position in motor rotations
     */
    public void setTargetPosition(double position) {
        targetPosition = position;
        setPositionControlFlag(true);
    }

    /**
     * Gets the current position of the intake motor
     * @return The current position in motor rotations
     */
    public double getCurrentPosition() {
        return intake.getPosition().getValueAsDouble();
    }

    /**
     * Sets the position control flag for the intake
     */
    public void setPositionControlFlag(boolean positionControl) {
        usePositionControl = positionControl;
    }
}