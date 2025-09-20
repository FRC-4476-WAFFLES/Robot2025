package frc.robot.subsystems.groundsuperstructure;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.data.Constants;
import frc.robot.data.Constants.GroundIntakeConstants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.utils.PhoenixHelpers;
import frc.robot.utils.IO.DeferredRefresher;
import frc.robot.utils.IO.TalonFXIO;
import frc.robot.utils.lib.SimpleWafflesMechanism;

/**
 * The GroundIntake subsystem handles the robot's L1 intake mechanism.
 * It controls:
 * - An intake motor for collecting game pieces
 */
public class GroundIntake extends SimpleWafflesMechanism {
    // Hardware Components
    private final TalonFXIO intakeLeft;
    private final TalonFXIO intakeRight;
    private final TalonFXIO intakeMid;

    private CANrange handoffCANRange;
    private LaserCan leftLaserCan;
    private LaserCan midLaserCan;
    private LaserCan rightLaserCan;
    
    // Sensor boilerplate
    private double leftLaserDistance = 9999;
    private double midLaserDistance = 9999;
    private double rightLaserDistance = 9999;
    private boolean handoffCoralPresent = false;
    
    private Trigger leftCoralSensor;
    private Trigger midCoralSensor;
    private Trigger rightCoralSensor;
    private Trigger handoffCoralSensor;

    // Deferred Refreshers
    private DeferredRefresher<Double> leftLaserCanRefresher = new DeferredRefresher<Double>(
        "Left Ground Intake LaserCAN", 
        0.02, // 50hz
        () -> {
            if (leftLaserCan != null) {
                var measurement = leftLaserCan.getMeasurement();
                if (measurement != null) {
                    if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                        return (double)measurement.distance_mm;
                    }
                }
            }
            return null;
        }
    );

    private DeferredRefresher<Double> midLaserCanRefresher = new DeferredRefresher<Double>(
        "Mid Ground Intake LaserCAN", 
        0.02, 
        () -> {
            if (midLaserCan != null) {
                var measurement = midLaserCan.getMeasurement();
                if (measurement != null) {
                    if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                        return (double)measurement.distance_mm;
                    }
                }
            } 
            return null;
        }
    );
    private DeferredRefresher<Double> rightLaserCanRefresher = new DeferredRefresher<Double>(
        "Right Ground Intake LaserCAN", 
        0.02, 
        () -> {
            if (rightLaserCan != null) {
                var measurement = rightLaserCan.getMeasurement();
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
    private final MotionMagicVelocityVoltage intakeRightControlRequest = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVelocityVoltage intakeLeftControlRequest = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVelocityVoltage intakeMidControlRequest = new MotionMagicVelocityVoltage(0);

    // State Variables
    public enum GroundIntakeState {
        SHIFT_LEFT(-1, -1,1),
        INTAKE_TOP(0, 0,5),
        INTAKE_TOP_SLOW(0, 0,0.5),
        SHIFT_RIGHT(1, 1,1),
        PREPARE_HANDOFF(-3,3,3),
        HANDOFF(3,-3,0),
        REST(0, 0,0),
        OUTAKE(-5,5,0),
        SPIT_OUT(5,-5,0);
        
        private final double rightSpeed;
        private final double leftSpeed;
        private final double topSpeed;
    
        GroundIntakeState(double rightSpeed, double leftSpeed, double topSpeed) {
          this.rightSpeed = rightSpeed;
          this.leftSpeed = leftSpeed;
          this.topSpeed = topSpeed;
        }

        public double getRightSpeed() {
          return rightSpeed;
        }
    
        public double getLeftSpeed() {
          return leftSpeed;
        }
        public double getTopSpeed() {
            return topSpeed;
        }
    }
    private GroundIntakeState currentState = GroundIntakeState.REST;

    // Network Tables
    private final BooleanPublisher handoffSensorNT = networkTable.getBooleanTopic("Handoff Sensor").publish();
    private final BooleanPublisher leftSensorNT = networkTable.getBooleanTopic("Left Sensor").publish();
    private final BooleanPublisher rightSensorNT = networkTable.getBooleanTopic("Right Sensor").publish();
    private final BooleanPublisher midSensorNT = networkTable.getBooleanTopic("Middle Sensor").publish();

    private final DoublePublisher rightIntakeSetpointNT = networkTable.getDoubleTopic("Right Intake Setpoint").publish();
    private final DoublePublisher leftIntakeSetpointNT = networkTable.getDoubleTopic("Left Intake Setpoint").publish();
    private final DoublePublisher midIntakeSetpointNT = networkTable.getDoubleTopic("Middle Intake Setpoint").publish();
    private final DoublePublisher rightIntakeVelocityNT = networkTable.getDoubleTopic("Right Intake Velocity").publish();
    private final DoublePublisher leftIntakeVelocityNT = networkTable.getDoubleTopic("Left Intake Velocity").publish();
    private final DoublePublisher midIntakeVelocityNT = networkTable.getDoubleTopic("Middle Intake Velocity").publish();
    
    private final DoublePublisher midSensorDistNT = networkTable.getDoubleTopic("Mid Distance").publish();
    private final DoublePublisher leftSensorDistNT = networkTable.getDoubleTopic("Left Distance").publish();
    private final DoublePublisher rightSensorDistNT = networkTable.getDoubleTopic("Right Distance").publish();
    
    public GroundIntake() {
        intakeRight = new TalonFXIO(Constants.CANIds.groundIntakeMotorRight);
        intakeLeft = new TalonFXIO(Constants.CANIds.groundIntakeMotorLeft);
        intakeMid = new TalonFXIO(Constants.CANIds.groundIntakeMotorMid);
        handoffCANRange = new CANrange(Constants.CANIds.groundIntakeCanRange);

        // Configure hardware
        configureLaserCAN();
        configureSideRollers();
        configureTopRoller();
        configureCANRange();
    }

    /**
     * Configures the CANRange
     */
    private void configureCANRange() {
        CANrangeConfiguration canRangeConfigs = new CANrangeConfiguration();
        canRangeConfigs.ProximityParams.ProximityThreshold = Constants.GroundIntakeConstants.CANRANGE_PROXIMITY_THRESHOLD;
        handoffCANRange.getConfigurator().apply(canRangeConfigs);
    }

    /**
     * Configures the laserCAN
     */
    private void configureLaserCAN() {
        // Initialize LaserCan with error handling
        try {
            leftLaserCan = new LaserCan(Constants.CANIds.groundIntakeLaserCanLeft);
            leftLaserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            leftLaserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
            
            midLaserCan = new LaserCan(Constants.CANIds.groundIntakeLaserCanMid);
            midLaserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            midLaserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);

            rightLaserCan = new LaserCan(Constants.CANIds.groundIntakeLaserCanRight);
            rightLaserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            rightLaserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
            
        } catch (Exception e) {
            // throw new RuntimeException("Failed to initialize LaserCan: " + e.getMessage());
            System.out.println("Failed to initialize LaserCan: " + e.getMessage());
            leftLaserCan = null;
            midLaserCan = null;
            rightLaserCan = null;
        }

        leftCoralSensor = new Trigger(
            () -> leftLaserDistance <= GroundIntakeConstants.CORAL_LEFT_DISTANCE_THRESHOLD
        ).debounce(GroundIntakeConstants.SENSOR_DEBOUNCE_TIME);

        midCoralSensor = new Trigger(
            () -> midLaserDistance <= GroundIntakeConstants.CORAL_MID_DISTANCE_THRESHOLD
        ).debounce(GroundIntakeConstants.SENSOR_DEBOUNCE_TIME);

        rightCoralSensor = new Trigger(
            () -> rightLaserDistance <= GroundIntakeConstants.CORAL_RIGHT_DISTANCE_THRESHOLD
        ).debounce(GroundIntakeConstants.SENSOR_DEBOUNCE_TIME);

        handoffCoralSensor = new Trigger(
            () -> handoffCoralPresent
        ).debounce(GroundIntakeConstants.SENSOR_DEBOUNCE_TIME);
    }

    /**
     * Configures the side roller motors
     */
    private void configureSideRollers() {
        TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
        CurrentLimitsConfigs intakeCurrentLimit = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(GroundIntakeConstants.STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true);


        intakeConfigs.CurrentLimits = intakeCurrentLimit;

        // Slot0 for velocity control
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.9;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kV = 1.2;
        slot0Configs.kG = 0.0;

        intakeConfigs.Slot0 = slot0Configs;

        intakeConfigs.Feedback.SensorToMechanismRatio = PhysicalConstants.groundIntakeSideRollersReduction;

        // Motion Magic
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration = 40;
        motionMagicConfigs.MotionMagicJerk = 0;
        intakeConfigs.MotionMagic = motionMagicConfigs;

        intakeConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeConfigs.MotorOutput.DutyCycleNeutralDeadband = 0.01;
        
        PhoenixHelpers.tryConfig(() -> intakeRight.getConfigurator().apply(intakeConfigs));
        PhoenixHelpers.tryConfig(() -> intakeLeft.getConfigurator().apply(intakeConfigs));
    }

    /**
     * Configures the top roller motor
     */
    private void configureTopRoller() {
        TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
        CurrentLimitsConfigs intakeCurrentLimit = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(GroundIntakeConstants.STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true);


        intakeConfigs.CurrentLimits = intakeCurrentLimit;

        // Slot0 for velocity control
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.9;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kV = 1.2;
        slot0Configs.kG = 0.0;

        intakeConfigs.Slot0 = slot0Configs;

        intakeConfigs.Feedback.SensorToMechanismRatio = PhysicalConstants.groundIntakeTopRollerReduction;

        // Motion Magic
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration = 40;
        motionMagicConfigs.MotionMagicJerk = 0;
        intakeConfigs.MotionMagic = motionMagicConfigs;

        intakeConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeConfigs.MotorOutput.DutyCycleNeutralDeadband = 0.01;

        PhoenixHelpers.tryConfig(() -> intakeMid.getConfigurator().apply(intakeConfigs));
    }
    
    @Override
    public void periodicImpl() {
        intakeRight.setControl(intakeRightControlRequest.withVelocity(currentState.getRightSpeed()).withSlot(0));
        intakeLeft.setControl(intakeLeftControlRequest.withVelocity(currentState.getLeftSpeed()).withSlot(0));
        intakeMid.setControl(intakeMidControlRequest.withVelocity(currentState.getTopSpeed()).withSlot(0));
        updateCoralSensors();
    }
    /**
     * Sets the target rotation of the ground intake.
     * @param setpoint Target rotation speed (GroundIntakeState enum)
     */
    public void setGroundIntakeSetpoint(GroundIntakeState state) {
        currentState = state;    
    }

    /**
     * Get the last defined setpoint the ground intake was set to
     * @return
     */
    public GroundIntakeState getSetpoint(){
        return currentState;
    }
    
    /**
     * Updates the coral sensor's internal state
     */
    private void updateCoralSensors() {
        if (RobotBase.isSimulation()) {
            leftLaserDistance = RobotContainer.telemetry.intakeSimLoaded ? 0 : 1000;
            midLaserDistance = RobotContainer.telemetry.intakeSimLoaded ? 0 : 1000;
            rightLaserDistance = RobotContainer.telemetry.intakeSimLoaded ? 0 : 1000;

            handoffCoralPresent = RobotContainer.telemetry.intakeHandoffSimLoaded;
            return;
        }

        var leftSensorResult = leftLaserCanRefresher.getLatestValue();
        if (leftSensorResult.isPresent()) {
            leftLaserDistance = leftSensorResult.get();
        }
        
        var midSensorResult = midLaserCanRefresher.getLatestValue();
        if (midSensorResult.isPresent()) {
            midLaserDistance = midSensorResult.get();
        }
        var rightSensorResult = rightLaserCanRefresher.getLatestValue();
        if (rightSensorResult.isPresent()) {
            rightLaserDistance = rightSensorResult.get();
        }

        handoffCoralPresent = handoffCANRange.getIsDetected().getValue();
    }

    public boolean isCoralLeft() {
        return leftCoralSensor.getAsBoolean();
    }

    public boolean isCoralMid() {
        return midCoralSensor.getAsBoolean();
    }

    public boolean isCoralRight() {
        return rightCoralSensor.getAsBoolean();
    }

    public boolean isCoralHandoffLoaded() {
        return handoffCoralSensor.getAsBoolean();
    }

    /**
     * This method is called automatically by the SubsystemNetworkManager
     */
    @Override
    public void updateNetwork() {
        rightIntakeSetpointNT.set(currentState.getRightSpeed());
        leftIntakeSetpointNT.set(currentState.getLeftSpeed());
        midIntakeSetpointNT.set(currentState.getTopSpeed());
        rightIntakeVelocityNT.set(intakeRight.signals().velocity().getValueAsDouble());
        leftIntakeVelocityNT.set(intakeLeft.signals().velocity().getValueAsDouble());
        midIntakeVelocityNT.set(intakeMid.signals().velocity().getValueAsDouble());


        midSensorDistNT.set(midLaserDistance);
        rightSensorDistNT.set(rightLaserDistance);
        leftSensorDistNT.set(leftLaserDistance);

        leftSensorNT.set(isCoralLeft());
        rightSensorNT.set(isCoralRight());
        midSensorNT.set(isCoralMid());
        handoffSensorNT.set(isCoralHandoffLoaded());
    }
}