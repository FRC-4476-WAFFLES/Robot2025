package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.data.Constants;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.data.Constants.SharkIntakeConstants;
import frc.robot.utils.NetworkUser;
import frc.robot.utils.SubsystemNetworkManager;

/**
 * The SharkIntake subsystem handles the robot's L1 intake mechanism.
 * It controls:
 * - An intake motor for collecting game pieces
 */
public class SharkIntake extends SubsystemBase implements NetworkUser{
    // Hardware Components
    private final TalonFX intake;

    // Control Objects
    private final MotionMagicVelocityVoltage intakeControlRequest = new MotionMagicVelocityVoltage(0);

    // State Variables
    private boolean coralLoaded = false;
    private double intakeSpeed = 0;
    private Trigger coralCurrentThresholdMet;
    private Trigger coralEjectionVelocityMet;

    // Debouncing variables for coral detection
    private static final double CORAL_DETECTION_DEBOUNCE_TIME = 0.2; // s
    private static final double CORAL_EJECTION_DEBOUNCE_TIME = 0.45; // s

    // Network Tables
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable intakeTable = inst.getTable("SharkIntake");
    private final BooleanPublisher coralLoadedNT = intakeTable.getBooleanTopic("Coral Loaded").publish();
    private final DoublePublisher intakeSetpointNT = intakeTable.getDoubleTopic("Intake Setpoint").publish();
    private final DoublePublisher intakeCurrentDrawNT = intakeTable.getDoubleTopic("Intake Current Draw").publish();
    private final DoublePublisher intakeVelocityNT = intakeTable.getDoubleTopic("Intake Velocity").publish();

    public SharkIntake() {
        SubsystemNetworkManager.RegisterNetworkUser(this, true, CodeConstants.SUBSYSTEM_NT_UPDATE_RATE);

        intake = new TalonFX(Constants.CANIds.sharkIntakeMotor);

        // Configure hardware
        configureIntakeMotor();

        // Construct debounced trigger
        coralCurrentThresholdMet = new Trigger(
            () -> intake.getTorqueCurrent().getValueAsDouble() > SharkIntakeConstants.CORAL_CURRENT_THRESHOLD
        ).debounce(CORAL_DETECTION_DEBOUNCE_TIME);

        coralEjectionVelocityMet = new Trigger(
            () -> intake.getVelocity().getValueAsDouble() < SharkIntakeConstants.CORAL_EJECT_VELOCITY_THRESHOLD
        ).debounce(CORAL_EJECTION_DEBOUNCE_TIME);
    }

    /**
     * Configures the intake motor with current limits
     */
    private void configureIntakeMotor() {
        TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
        CurrentLimitsConfigs intakeCurrentLimit = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(SharkIntakeConstants.STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true);


        intakeConfigs.CurrentLimits = intakeCurrentLimit;

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.9;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kV = 1.2;
        slot0Configs.kG = 0.0;

        intakeConfigs.Slot0 = slot0Configs;

        intakeConfigs.Feedback.SensorToMechanismRatio = PhysicalConstants.sharkIntakeReduction;

        // Motion Magic
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration = 40;
        motionMagicConfigs.MotionMagicJerk = 0;
        intakeConfigs.MotionMagic = motionMagicConfigs;

        intakeConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeConfigs.MotorOutput.DutyCycleNeutralDeadband = 0.01;

        intake.getConfigurator().apply(intakeConfigs);
    }
    
    @Override
    public void periodic() {
        intake.setControl(intakeControlRequest.withVelocity(intakeSpeed).withSlot(0));

        detectCoralLoaded();
    }

    /**
     * Sets the intake motor speed
     * @param speed Speed value (rotations/s)
     */
    public void setIntakeSpeed(double speed) {
        this.intakeSpeed = speed;
    }

    /**
     * Checks if coral is present in the intake based on current draw
     * @return true if coral is detected
     */
    public void detectCoralLoaded() {
        if (coralCurrentThresholdMet.getAsBoolean()) {
            coralLoaded = true;
        } else if (coralEjectionVelocityMet.getAsBoolean()) {
            coralLoaded = false;
        }
    }

    public boolean isCoralLoaded() {
        return coralLoaded;
    }

    /**
     * This method is called automatically by the SubsystemNetworkManager
     */
    @Override
    public void updateNetwork() {
        coralLoadedNT.set(isCoralLoaded());
        intakeSetpointNT.set(intakeSpeed);
        intakeCurrentDrawNT.set(intake.getTorqueCurrent().getValueAsDouble());
        intakeVelocityNT.set(intake.getVelocity().getValueAsDouble());
    }

    @Override
    public void initializeNetwork() {
        // Network initialization if needed
    }
}