package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.data.Constants;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.ManipulatorConstants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.utils.PhoenixHelpers;
import frc.robot.utils.IO.DeferredRefresher;
import frc.robot.utils.IO.TalonFXIO;
import frc.robot.utils.lib.SimpleWafflesMechanism;

/**
 * The Intake subsystem handles the robot's intake mechanism.
 * It controls:
 * - An intake motor for collecting game pieces
 * - A LaserCan sensor for detecting game pieces
 */
public class Intake extends SimpleWafflesMechanism {
    // Hardware Components
    private final TalonFXIO intake;
    private LaserCan intakeLaserCan;
    private final DigitalInput coralSensor;

    // Deferred Refreshers
    private DeferredRefresher<Double> intakeLaserCanRefresher = new DeferredRefresher<Double>(
        "Intake LaserCAN", 
        0.02, // 50hz
        () -> {
            if (intakeLaserCan != null) {
                var measurement = intakeLaserCan.getMeasurement();
                if (measurement != null) {
                    if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                        return (double)measurement.distance_mm;
                    }
                }
            }
            return null;
        }
    );

    // Control Objects
    private final MotionMagicVelocityVoltage intakeControlRequest = new MotionMagicVelocityVoltage(0);
    private final VoltageOut intakePositionRequest = new VoltageOut(0).withEnableFOC(true);
    // private final PositionVoltage intakePositionControlRequest = new PositionVoltage(0).withSlot(1);

    // State Variables
    private double intakeLaserDistance = 0;
    private double intakeSpeed = 0;

    private boolean noAlgaeFlag = false;
    private boolean algaeLoaded = false;
    private double dutyCycle = 0;

    private Trigger algaeDetectionTrigger;

    // Network Tables
    private final DoublePublisher intakeLaserCanDistanceNT = networkTable.getDoubleTopic("Intake Laser Distance (mm)").publish();
    private final BooleanPublisher coralLoadedNT = networkTable.getBooleanTopic("Coral Loaded").publish();
    private final BooleanPublisher algaeLoadedNT = networkTable.getBooleanTopic("Algae Loaded").publish();
    private final DoublePublisher intakeSetpointNT = networkTable.getDoubleTopic("Intake Setpoint").publish();
    private final DoublePublisher intakeCurrentDrawNT = networkTable.getDoubleTopic("Intake Current Draw").publish();
    private final DoublePublisher intakeVelocityNT = networkTable.getDoubleTopic("Intake Velocity").publish();
    private final BooleanPublisher coralSensorRawNT = networkTable.getBooleanTopic("Coral Sensor Raw").publish();

    private final BooleanPublisher isIntakingAlgaeNT = networkTable.getBooleanTopic("IsIntaking").publish();
    private final BooleanPublisher isOutakingAlgaeNT = networkTable.getBooleanTopic("IsOutaking").publish();

    public Intake() {
        intake = new TalonFXIO(Constants.CANIds.intakeMotor);
        coralSensor = new DigitalInput(Constants.DigitalOutputs.coralSensor);

        // Configure hardware
        configureIntakeMotor();
        configureLaserCAN();

        algaeDetectionTrigger = new Trigger(
            () -> intake.signals().statorCurrent().getValueAsDouble() > ManipulatorConstants.ALGAE_CURRENT_THRESHOLD 
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
        } catch (Exception e) {
            // throw new RuntimeException("Failed to initialize LaserCan: " + e.getMessage());
            System.out.println("Failed to initialize LaserCan: " + e.getMessage());
            intakeLaserCan = null;
        }
    }

    /**
     * Configures the intake motor with current limits
     */
    private void configureIntakeMotor() {
        TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
        CurrentLimitsConfigs intakeCurrentLimit = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(80)
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

        PhoenixHelpers.tryConfig(() -> intake.getConfigurator().apply(intakeConfigs));
    }
    
    @Override
    public void periodicImpl() {
        if (Math.abs(dutyCycle) > 0.01) {
            intake.set(dutyCycle);
            
        } else {
            if (Math.abs(intakeSpeed) < 0.01 && isAlgaeLoaded()) {
                // When algae is loaded, run intake slowly inward
                intake.setControl(intakeControlRequest.withVelocity(Constants.ManipulatorConstants.ALGAE_HOLD_SPEED).withSlot(0));

                if (intake.signals().statorCurrent().getValueAsDouble() < 4) {
                    intake.setControl(intakeControlRequest.withVelocity(-120).withSlot(0));
                }
            } else if (Math.abs(intakeSpeed) < 0.01 && isCoralLoaded()) {
                intake.setControl(intakePositionRequest.withOutput(0)); // scuffed
            } else {
                intake.setControl(intakeControlRequest.withVelocity(intakeSpeed).withSlot(0));
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

        } else if (isOuttakingAlgae()) {
            algaeLoaded = false;
        }
    }

    /**
     * Updates the coral sensor's internal state
     */
    private void updateCoralSensors() {
        if (RobotBase.isSimulation()) {
            intakeLaserDistance = RobotContainer.telemetry.manipulatorSimLoaded ? 0 : 1000;
            return;
        }

        var intakeSensorResult = intakeLaserCanRefresher.getLatestValue();
        if (intakeSensorResult.isPresent()) {
            intakeLaserDistance = intakeSensorResult.get();
        }
    }

    /**
     * Is algae loaded in the manipulator
     * @return a boolean
     */
    public boolean isAlgaeLoaded() {
        if (RobotBase.isSimulation()) {
            // algae override for sim
            return CodeConstants.FORCE_LOAD_SIM_ALGAE;
        }

        return algaeLoaded;
    }

    /**
     * Checks if coral is loaded using the digital sensor
     * @return true if coral is detected
     */
    public boolean isCoralLoaded() {
        // return !coralSensor.get(); // Digital input is inverted (true when not pressed, false when pressed)
        return intakeLaserDistance <= Constants.ManipulatorConstants.CORAL_LOADED_DISTANCE_THRESHOLD;
    }

    /* Helper methods for determining the intake's basic state */

    public boolean isIntakingAlgae() {
        return !isAlgaeLoaded() && intakeSpeed > 10 && !noAlgaeFlag;
    }

    public boolean isOuttakingAlgae() {
        return isAlgaeLoaded() && intake.signals().velocity().getValueAsDouble() < -12;
    }

    public boolean isIntakeStopped() {
        return Math.abs(intake.signals().velocity().getValueAsDouble()) < 0.1;
    }

    /**
     * This method is called automatically by the SubsystemNetworkManager
     */
    @Override
    public void updateNetwork() {
        intakeLaserCanDistanceNT.set(intakeLaserDistance);
        coralLoadedNT.set(isCoralLoaded());
        algaeLoadedNT.set(isAlgaeLoaded());
        intakeSetpointNT.set(intakeSpeed);
        intakeCurrentDrawNT.set(intake.signals().statorCurrent().getValueAsDouble());
        intakeVelocityNT.set(intake.signals().velocity().getValueAsDouble());
        coralSensorRawNT.set(coralSensor.get());

        isIntakingAlgaeNT.set(isIntakingAlgae());
        isOutakingAlgaeNT.set(isOuttakingAlgae());
    }

    /**
     * Gets the current position of the intake motor
     * @return The current position in motor rotations
     */
    public double getCurrentPosition() {
        return intake.signals().position().getValueAsDouble();
    }

    /*
     * Apply duty cycle
     */
    public void setDutyCycle(double dutyCycleval) {
        dutyCycle = dutyCycleval;
    }
}