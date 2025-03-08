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
import frc.robot.data.Constants;
import frc.robot.data.Constants.CodeConstants;
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
    private LaserCan laserCan;

    // Control Objects
    private final MotionMagicVelocityVoltage intakeControlRequest = new MotionMagicVelocityVoltage(0);
    private final VoltageOut intakePositionRequest = new VoltageOut(0).withEnableFOC(true);
    private final PositionVoltage intakePositionControlRequest = new PositionVoltage(0).withSlot(1);

    // State Variables
    private double laserCANDistance = 0;
    private boolean algaeLoaded = false;
    private double intakeSpeed = 0;
    private double lastPosition = 0;
    private double targetPosition = 0;
    private boolean usePositionControl = false;
    private boolean noAlgaeFlag = false;

    // Debouncing variables for algae detection
    private long algaeDetectionStartTime = 0;
    private static final long ALGAE_DETECTION_DEBOUNCE_TIME = 100; // 100ms debounce time

    private Timer algaeLossTimer = new Timer();

    // Network Tables
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable intakeTable = inst.getTable("Intake");
    private final DoublePublisher laserCanDistanceNT = intakeTable.getDoubleTopic("LaserCan Distance (mm)").publish();
    private final BooleanPublisher coralLoadedNT = intakeTable.getBooleanTopic("Coral Loaded").publish();
    private final BooleanPublisher algeaLoadedNT = intakeTable.getBooleanTopic("Algea Loaded").publish();
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
    }

    /**
     * Configures the laserCAN
     */
    private void configureLaserCAN() {
        // Initialize LaserCan with error handling
        try {
            laserCan = new LaserCan(Constants.CANIds.laserCan);
            laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            // laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        } catch (Exception e) {
            // throw new RuntimeException("Failed to initialize LaserCan: " + e.getMessage());
            System.out.println("Failed to initialize LaserCan: " + e.getMessage());
            laserCan = null;
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
        slot0Configs.kV = 0.2;
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

        if (Math.abs(intakeSpeed) < 0.01 && isAlgaeLoaded()) {
            // When algae is loaded, run intake slowly inward
            intake.setControl(intakeControlRequest.withVelocity(Constants.ManipulatorConstants.ALGAE_HOLD_SPEED).withSlot(0));

            if (intake.getStatorCurrent().getValueAsDouble() < 4) {
                intake.setControl(intakeControlRequest.withVelocity(-300).withSlot(0));
                
                if (!algaeLossTimer.isRunning()) {
                    algaeLossTimer.reset();
                    algaeLossTimer.start();
                }
            }
        } else if (usePositionControl && isCoralLoaded()) {
            // Use position control to maintain coral position
            intake.setControl(intakePositionControlRequest
                .withPosition(targetPosition));
        } else if (Math.abs(intakeSpeed) < 0.01 && isCoralLoaded()) {
            // Hold position when speed is near zero
            double currentPosition = intake.getPosition().getValueAsDouble();
            lastPosition = currentPosition;
            intake.setControl(intakePositionRequest.withOutput(0));
        } else {
            intake.setControl(intakeControlRequest.withVelocity(intakeSpeed).withSlot(0));
        }

        detectAlgaeLoaded();
        updateCoralSensor();
    }

    /**
     * Sets the intake motor speed
     * @param speed Speed value (rotations/s)
     */
    public void setIntakeSpeed(double speed) {
        this.intakeSpeed = speed;
    }

    public void setNoAlgaeFlag(boolean val) {
        noAlgaeFlag = val;
    }

    /**
     * Checks if algae is present in the intake based on current draw
     * @return true if algae is detected
     */
    public void detectAlgaeLoaded() {
        boolean currentThresholdMet = intake.getStatorCurrent().getValueAsDouble() > Constants.ManipulatorConstants.ALGAE_CURRENT_THRESHOLD 
            && isIntakingAlgae() && !isCoralLoaded();

        if (currentThresholdMet) {
            // If we haven't started timing yet, start now
            if (algaeDetectionStartTime == 0) {
                algaeDetectionStartTime = System.currentTimeMillis();
            }
            // Check if we've exceeded the debounce time
            else if (System.currentTimeMillis() - algaeDetectionStartTime >= ALGAE_DETECTION_DEBOUNCE_TIME) {
                algaeLoaded = true;
                algaeLossTimer.stop();
                algaeLossTimer.reset();
            }
        } else {
            // Reset the timer if conditions aren't met
            algaeDetectionStartTime = 0;
            if (isOuttakingAlgae()) {
                algaeLoaded = false;
            }
        }
    }

    public boolean isAlgaeLoaded() {
        return algaeLoaded;
        // return Constants.ManipulatorConstants.ALGAE_LOADED_DISTANCE_UPPER_LIMIT >= laserCANDistance && laserCANDistance >= Constants.ManipulatorConstants.ALGAE_LOADED_DISTANCE_THRESHOLD;
    }

    /**
     * Checks if coral is loaded using the laser distance sensor
     * @return true if coral is detected within threshold distance
     */
    public boolean isCoralLoaded() {
        return laserCANDistance <= Constants.ManipulatorConstants.CORAL_LOADED_DISTANCE_THRESHOLD;
    }

    private void updateCoralSensor() {
        if (laserCan != null) {
            var measurement = laserCan.getMeasurement();
            if (measurement != null) {
                if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                    laserCANDistance = measurement.distance_mm;
                }
            }
        }
    }

    public boolean isIntakingCoral() {
        return !isCoralLoaded() && intake.getVelocity().getValueAsDouble() > 0;
    }

    public boolean isIntakingAlgae() {
        return !isAlgaeLoaded() && intakeSpeed > 10 && !noAlgaeFlag;
    }

    public boolean isOuttakingAlgae() {
        return isAlgaeLoaded() && intake.getVelocity().getValueAsDouble() < -12;
    }

    public boolean isOuttakingCoral() {
        return isCoralLoaded() && intake.getVelocity().getValueAsDouble() < 0;
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
        laserCanDistanceNT.set(laserCANDistance);
        coralLoadedNT.set(isCoralLoaded());
        algeaLoadedNT.set(isAlgaeLoaded());
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
        usePositionControl = true;
    }

    /**
     * Gets the current position of the intake motor
     * @return The current position in motor rotations
     */
    public double getCurrentPosition() {
        return intake.getPosition().getValueAsDouble();
    }

    /**
     * Disables position control and returns to velocity control
     */
    public void disablePositionControl() {
        usePositionControl = false;
    }
}