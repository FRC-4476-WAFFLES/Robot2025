package frc.robot.subsystems.GroundSuperstructure;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANrange;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.data.Constants;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.data.Constants.GroundIntakeConstants;
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

    private CANrange handoffCANRange = new CANrange(Constants.CANIds.groundIntakeCanRange);
    private LaserCan leftLaserCan;
    private LaserCan midLaserCan;
    private LaserCan rightLaserCan;
    
    // Sensor boilerplate
    private double leftLaserDistance = 0;
    private double midLaserDistance = 0;
    private double rightLaserDistance = 0;
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
        SHIFT_LEFT(0, 0,20),
        INTAKE_TOP(0, 0,20),
        SHIFT_RIGHT(0, 0,20),
        PREPARE_HANDOFF(10,-10,20),
        HANDOFF(0,0,0),
        REST(0, 0,0),
        OUTAKE(10,-10,0);
        
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
    private final BooleanPublisher coralLoadedNT = networkTable.getBooleanTopic("Coral Loaded").publish();
    private final DoublePublisher rightIntakeSetpointNT = networkTable.getDoubleTopic("Right Intake Setpoint").publish();
    private final DoublePublisher leftIntakeSetpointNT = networkTable.getDoubleTopic("Left Intake Setpoint").publish();
    private final DoublePublisher midIntakeSetpointNT = networkTable.getDoubleTopic("Middle Intake Setpoint").publish();
    private final DoublePublisher rightIntakeVelocityNT = networkTable.getDoubleTopic("Right Intake Velocity").publish();
    private final DoublePublisher leftIntakeVelocityNT = networkTable.getDoubleTopic("Left Intake Velocity").publish();
    private final DoublePublisher midIntakeVelocityNT = networkTable.getDoubleTopic("Middle Intake Velocity").publish();
    
    public GroundIntake() {
        intakeRight = new TalonFXIO(Constants.CANIds.groundIntakeMotorRight);
        intakeLeft = new TalonFXIO(Constants.CANIds.groundIntakeMotorLeft);
        intakeMid = new TalonFXIO(Constants.CANIds.groundIntakeMotorMid);
        // Configure hardware
        CANrangeConfiguration canRangeConfigs = new CANrangeConfiguration();
        canRangeConfigs.ProximityParams.ProximityThreshold = Constants.GroundIntakeConstants.CANRANGE_PROXIMITY_THRESHOLD;
        handoffCANRange.getConfigurator().apply(canRangeConfigs);
        configureLaserCAN();
        configureIntakeMotors();
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
     * Configures the intake motor with current limits
     */
    private void configureIntakeMotors() {
        TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
        CurrentLimitsConfigs intakeCurrentLimit = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(GroundIntakeConstants.STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true);


        intakeConfigs.CurrentLimits = intakeCurrentLimit;

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.9;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kV = 1.2;
        slot0Configs.kG = 0.0;

        intakeConfigs.Slot0 = slot0Configs;

        intakeConfigs.Feedback.SensorToMechanismRatio = PhysicalConstants.groundIntakeReduction;

        // Motion Magic
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration = 40;
        motionMagicConfigs.MotionMagicJerk = 0;
        intakeConfigs.MotionMagic = motionMagicConfigs;

        intakeConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeConfigs.MotorOutput.DutyCycleNeutralDeadband = 0.01;
        PhoenixHelpers.tryConfig(() -> intakeMid.getConfigurator().apply(intakeConfigs));
        PhoenixHelpers.tryConfig(() -> intakeRight.getConfigurator().apply(intakeConfigs));
        PhoenixHelpers.tryConfig(() -> intakeLeft.getConfigurator().apply(intakeConfigs));
    }
    
    @Override
    public void periodicImpl() {
        intakeRight.setControl(intakeRightControlRequest.withVelocity(currentState.getRightSpeed()).withSlot(0));
        intakeLeft.setControl(intakeLeftControlRequest.withVelocity(currentState.getLeftSpeed()).withSlot(0));//not sure if they will be following same speed 
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
   * Get the last defined rotation setpoint the ground intake was set to
   * @return
   */
    public GroundIntakeState getGroundIntakeSetpointEnum(){
        return currentState;
    }
    /**
     * Updates the coral sensor's internal state
     */
    private void updateCoralSensors() {
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
        coralLoadedNT.set(isCoralHandoffLoaded());
        rightIntakeSetpointNT.set(currentState.getRightSpeed());
        leftIntakeSetpointNT.set(currentState.getLeftSpeed());
        midIntakeSetpointNT.set(currentState.getTopSpeed());
        rightIntakeVelocityNT.set(intakeRight.signals().velocity().getValueAsDouble());
        leftIntakeVelocityNT.set(intakeLeft.signals().velocity().getValueAsDouble());
        midIntakeVelocityNT.set(intakeMid.signals().velocity().getValueAsDouble());
        coralLoadedNT.set(isCoralHandoffLoaded());
    }
}