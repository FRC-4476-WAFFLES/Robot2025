package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Controls;
import frc.robot.RobotContainer;
import frc.robot.data.Constants;
import frc.robot.data.Constants.ManipulatorConstants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.subsystems.DynamicPathing.DynamicPathingSituation;
import frc.robot.utils.PhoenixHelpers;
import frc.robot.utils.IO.TalonFXIO;
import frc.robot.utils.lib.SimpleWafflesMechanism;

/**
 * The Intake subsystem handles the robot's intake mechanism.
 * It controls:
 * - An intake motor for collecting game pieces
 */
public class Intake extends SimpleWafflesMechanism {
    // Hardware Components
    private final TalonFXIO intake;

    // Control Objects
    private final MotionMagicVelocityVoltage intakeControlRequest = new MotionMagicVelocityVoltage(0);
    private final VoltageOut intakePositionRequest = new VoltageOut(0).withEnableFOC(true);

    // State Variables
    private double intakeSpeed = 0;
    private boolean manipulatorLoaded = false;

    private boolean noAlgaeFlag = false;
    private double dutyCycle = 0;

    private Trigger algaeDetectionTrigger;
    private Trigger coralDetectionTrigger;
    private Trigger coralReleaseTrigger;

    private enum LoadType {
        ALGEA,
        CORAL;
    }
    private LoadType loadType = LoadType.CORAL;

    // Network Tables
    private final BooleanPublisher coralLoadedNT = networkTable.getBooleanTopic("Coral Loaded").publish();
    private final BooleanPublisher algaeLoadedNT = networkTable.getBooleanTopic("Algae Loaded").publish();
    private final BooleanPublisher manipulatorLoadedNT = networkTable.getBooleanTopic("Manipulator Loaded").publish();
    private final StringPublisher loadTypeNT = networkTable.getStringTopic("Load Type").publish();
    private final DoublePublisher intakeSetpointNT = networkTable.getDoubleTopic("Intake Setpoint").publish();
    private final DoublePublisher intakeCurrentDrawNT = networkTable.getDoubleTopic("Intake Current Draw").publish();

    private final BooleanPublisher isIntakingAlgaeNT = networkTable.getBooleanTopic("IsIntaking").publish();
    private final BooleanPublisher isOutakingAlgaeNT = networkTable.getBooleanTopic("IsOutaking").publish();

    public Intake() {
        intake = new TalonFXIO(Constants.CANIds.manipulatorIntake);

        // Configure hardware
        configureIntakeMotor();

        algaeDetectionTrigger = new Trigger(
            () -> intake.signals().statorCurrent().getValueAsDouble() > ManipulatorConstants.ALGAE_CURRENT_THRESHOLD 
            && loadType == LoadType.ALGEA
            && !isAlgaeLoaded()
        ).debounce(ManipulatorConstants.ALGAE_DETECTION_DEBOUNCE_TIME);

        coralDetectionTrigger = new Trigger(
            () -> intake.signals().statorCurrent().getValueAsDouble() > ManipulatorConstants.CORAL_CURRENT_THRESHOLD 
            && loadType == LoadType.CORAL
            && !isCoralLoaded()
        ).debounce(ManipulatorConstants.CORAL_DETECTION_DEBOUNCE_TIME);

        coralReleaseTrigger = new Trigger(
            () -> isOuttakingCoral()
        ).debounce(ManipulatorConstants.CORAL_RELEASE_DEBOUNCE_TIME);
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
        slot0Configs.kP = 1;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kV = 1.5;
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
        // Determine intake state
        if (!manipulatorLoaded) {
            // Only change load type while not loaded
            if (RobotContainer.groundSuperstructure.isHandoffHappening()) {
                loadType = LoadType.CORAL;
            } else if (
                RobotContainer.dynamicPathingSubsystem.getCurrentPathingSituation() == DynamicPathingSituation.REEF_ALGAE &&
                RobotContainer.dynamicPathingSubsystem.runningAction.getAsBoolean()
            ) {
                loadType = LoadType.ALGEA;
            } else if (Controls.operatorController.povDown().getAsBoolean()) {
                // Quick hack
                loadType = LoadType.ALGEA;
            } 
            // else if (RobotContainer.isGroundIntakingAlgae) {
            //     loadType = LoadType.ALGEA;
            // }
        }

        // Update gamepeice sensing
        detectGamepeiceLoaded();
        
        // Run motor
        if (Math.abs(dutyCycle) > 0.01) {
            intake.set(dutyCycle);
        } else {
            if (Math.abs(intakeSpeed) < 0.01 && isAlgaeLoaded()) {
                // When algae is loaded, run intake slowly inward
                intake.setControl(intakeControlRequest.withVelocity(Constants.ManipulatorConstants.ALGAE_HOLD_SPEED).withSlot(0));
            } else if (Math.abs(intakeSpeed) < 0.01 && isCoralLoaded()) {
                intake.setControl(intakePositionRequest.withOutput(0)); // scuffed
            } else {
                intake.setControl(intakeControlRequest.withVelocity(intakeSpeed).withSlot(0));
            }
        }
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
     * Checks if algae or coral is present in the intake based on current draw
     */
    private void detectGamepeiceLoaded() {
        if (RobotBase.isSimulation()) {
            return;
        }

        if (loadType == LoadType.ALGEA) {
            if (algaeDetectionTrigger.getAsBoolean()) {
                manipulatorLoaded = true;
    
            } else if (isOuttakingAlgae()) {
                manipulatorLoaded = false;
            }
        } else {
            if (coralDetectionTrigger.getAsBoolean()) {
                manipulatorLoaded = true;
    
            } else if (coralReleaseTrigger.getAsBoolean()) {
                manipulatorLoaded = false;
            }
        }
    }

    public void forceLoadCoral() {
        loadType = LoadType.CORAL;
        manipulatorLoaded = true;

        // sim
        if (RobotBase.isSimulation()) {
            RobotContainer.telemetry.manipulatorCoralSimLoaded = true;
        }
    }

    /**
     * Is algae loaded in the manipulator
     * @return a boolean
     */
    public boolean isAlgaeLoaded() {
        if (RobotBase.isSimulation()) {
            // algae override for sim
            return RobotContainer.telemetry.algeaSimLoaded;
        }

        return loadType == LoadType.ALGEA && manipulatorLoaded;
    }

    /**
     * Checks if coral is loaded using the digital sensor
     * @return true if coral is detected
     */
    public boolean isCoralLoaded() {
        if (RobotBase.isSimulation()) {
            // coral override for sim
            return RobotContainer.telemetry.manipulatorCoralSimLoaded;
        }

        return loadType == LoadType.CORAL && manipulatorLoaded;
    }

    /**
     * Checks if any gamepeice is occupying the intake
     * @return true if a gamepeice is loaded
     */
    public boolean manipulatorLoaded() {
        if (RobotBase.isSimulation()) {
            return RobotContainer.telemetry.manipulatorCoralSimLoaded || RobotContainer.telemetry.algeaSimLoaded;
        }

        return manipulatorLoaded;
    }

    /* Helper methods for determining the intake's basic state */

    // public boolean isIntakingAlgae() {
    //     return !isAlgaeLoaded() && intakeSpeed > 10 && !noAlgaeFlag;
    // }

    public boolean isOuttakingAlgae() {
        return isAlgaeLoaded() && intake.signals().velocity().getValueAsDouble() > 0.5;
    }

    public boolean isOuttakingCoral() {
        return isCoralLoaded() && intake.signals().velocity().getValueAsDouble() > 0.5;
    }

    public boolean isIntakeStopped() {
        return Math.abs(intake.signals().velocity().getValueAsDouble()) < 0.1;
    }

    /**
     * This method is called automatically by the SubsystemNetworkManager
     */
    @Override
    public void updateNetwork() {
        coralLoadedNT.set(isCoralLoaded());
        algaeLoadedNT.set(isAlgaeLoaded());
        manipulatorLoadedNT.set(manipulatorLoaded());
        loadTypeNT.set(loadType.toString());
        intakeSetpointNT.set(intakeSpeed);
        intakeCurrentDrawNT.set(intake.signals().statorCurrent().getValueAsDouble());

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