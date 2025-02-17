package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.RobotContainer.pivotSubsystem;
import frc.robot.data.Constants;
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

    // State Variables
    private double laserCANDistance = 0;
    private boolean algaeLoaded = false;
    private double intakeSpeed = 0;
    private double lastPosition = 0;

    // Network Tables
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable pivotTable = inst.getTable("Intake");
    private final DoublePublisher laserCanDistanceNT = pivotTable.getDoubleTopic("LaserCan Distance (mm)").publish();
    private final BooleanPublisher coralLoadedNT = pivotTable.getBooleanTopic("Coral Loaded").publish();
    private final BooleanPublisher algeaLoadedNT = pivotTable.getBooleanTopic("Algea Loaded").publish();
    private final DoublePublisher intakeSetpointNT = pivotTable.getDoubleTopic("Intake Setpoint").publish();
    private final DoublePublisher intakeCurrentDrawNT = pivotTable.getDoubleTopic("Intake Current Draw").publish();

    public Intake() {
        SubsystemNetworkManager.RegisterNetworkUser(this);

        intake = new TalonFX(Constants.CANIds.intakeMotor);

        // Configure hardware
        configureIntakeMotor();
        configureLaserCAN();
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

        intakeConfigs.Slot0 = slot0Configs;

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
        if (Math.abs(intakeSpeed) < 0.01 && isAlgaeLoaded()) {
            // When algae is loaded, run intake slowly inward
            intake.setControl(intakeControlRequest.withVelocity(Constants.ManipulatorConstants.ALGAE_HOLD_SPEED).withSlot(0));
        } else if (Math.abs(intakeSpeed) < 0.01 && isCoralLoaded()) {
            // Hold position when speed is near zero
            double currentPosition = intake.getPosition().getValueAsDouble();
            lastPosition = currentPosition;
            intake.setControl(intakePositionRequest.withOutput(0));
        } else {
            intake.setControl(intakeControlRequest.withVelocity(intakeSpeed).withSlot(0));
        }

        updateCoralSensor();
    }
    /**
     * Sets the intake motor speed
     * @param speed Speed value between -1.0 and 1.0
     */
    public void setIntakeSpeed(double speed) {
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
        return Constants.ManipulatorConstants.ALGAE_LOADED_DISTANCE_UPPER_LIMIT >= laserCANDistance && laserCANDistance >= Constants.ManipulatorConstants.ALGAE_LOADED_DISTANCE_THRESHOLD;
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
        return !isAlgaeLoaded() && intake.getVelocity().getValueAsDouble() < 0;
    }

    public boolean isOuttakingAlgae() {
        return isAlgaeLoaded() && intake.getVelocity().getValueAsDouble() > 0;
    }

    public boolean isOuttakingCoral() {
        return isCoralLoaded() && intake.getVelocity().getValueAsDouble() < 0;
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
    }

    @Override
    public void initializeNetwork() {
        // Network initialization if needed
    }
}