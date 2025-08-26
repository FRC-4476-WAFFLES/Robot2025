package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.data.Constants;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.data.Constants.GroundIntakeConstants;
import frc.robot.utils.NetworkUser;
import frc.robot.utils.PhoenixHelpers;
import frc.robot.utils.SubsystemNetworkManager;
import frc.robot.utils.IO.DeferredRefresher;
import frc.robot.utils.IO.TalonFXIO;
import frc.robot.utils.lib.WafflesMechanism;

/**
 * The GroundIntake subsystem handles the robot's L1 intake mechanism.
 * It controls:
 * - An intake motor for collecting game pieces
 */
public class GroundIntake extends WafflesMechanism implements NetworkUser{
    // Hardware Components
    private final TalonFXIO intakeLeft;
    private final TalonFXIO intakeRight;
    private final TalonFXIO intakeMid;
    private CANrange CANrange = new CANrange(Constants.CANIds.groundIntakeCanRange);
    private LaserCan leftLaserCan;
    private LaserCan midLaserCan;
    private LaserCan rightLaserCan;
    private double leftLaserDistance = 0;
    private double midLaserDistance = 0;
    private double rightLaserDistance = 0;
    // Deferred Refreshers
    private DeferredRefresher<Double> leftLaserCanRefresher = new DeferredRefresher<Double>(
        "Left Ground Intake LaserCAN", 
        0.01, // 100hz
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
        0.01, 
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
        0.01, 
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
        //TODO make actual states depending on what we want to do
        SHIFT_LEFT(0, 0,20),
        INTAKE_MID(0, 0,20),
        SHIFT_RIGHT(0, 0,20),
        STASH(0, 0,20),
        FEED(0, 0,20),
        REST(0, 0,0),
        OUTAKE(150,0.33,0);
        
        private final double rightSpeed;
        private final double leftSpeed;
        private final double midSpeed;
    
        GroundIntakeState(double rightSpeed, double leftSpeed, double midSpeed) {
          this.rightSpeed = rightSpeed;
          this.leftSpeed = leftSpeed;
          this.midSpeed = midSpeed;
        }

        public double getRightSpeed() {
          return rightSpeed;
        }
    
        public double getLeftSpeed() {
          return leftSpeed;
        }
        public double getMidSpeed() {
            return midSpeed;
        }
    }
    private GroundIntakeState currentState = GroundIntakeState.INTAKE_MID;
    private boolean coralInRange=false;

    // Network Tables
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable intakeTable = inst.getTable("GroundIntake");
    private final BooleanPublisher coralLoadedNT = intakeTable.getBooleanTopic("Coral Loaded").publish();
    private final DoublePublisher rightIntakeSetpointNT = intakeTable.getDoubleTopic("Right Intake Setpoint").publish();
    private final DoublePublisher leftIntakeSetpointNT = intakeTable.getDoubleTopic("Left Intake Setpoint").publish();
    private final DoublePublisher midIntakeSetpointNT = intakeTable.getDoubleTopic("Middle Intake Setpoint").publish();
    private final DoublePublisher rightIntakeVelocityNT = intakeTable.getDoubleTopic("Right Intake Velocity").publish();
    private final DoublePublisher leftIntakeVelocityNT = intakeTable.getDoubleTopic("Left Intake Velocity").publish();
    private final DoublePublisher midIntakeVelocityNT = intakeTable.getDoubleTopic("Middle Intake Velocity").publish();
    
    public GroundIntake() {
        SubsystemNetworkManager.RegisterNetworkUser(this, true, CodeConstants.SUBSYSTEM_NT_UPDATE_RATE);

        intakeRight = new TalonFXIO(Constants.CANIds.groundIntakeMotorRight);
        intakeLeft = new TalonFXIO(Constants.CANIds.groundIntakeMotorLeft);
        intakeMid = new TalonFXIO(Constants.CANIds.groundIntakeMotorMid);
        // Configure hardware
        CANrangeConfiguration canRangeConfigs = new CANrangeConfiguration();
        canRangeConfigs.ProximityParams.ProximityThreshold = Constants.GroundIntakeConstants.CANRANGE_PROXIMITY_THRESHOLD;
        CANrange.getConfigurator().apply(canRangeConfigs);
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
        intakeMid.setControl(intakeMidControlRequest.withVelocity(currentState.getMidSpeed()).withSlot(0));
        updateCoralSensors();
        isCoralLoaded();
        
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
    }

    public boolean isCoralLeft() {
        return leftLaserDistance <= Constants.GroundIntakeConstants.CORAL_LEFT_DISTANCE_THRESHOLD;
    }

    public boolean isCoralMid() {
        return midLaserDistance <= Constants.GroundIntakeConstants.CORAL_MID_DISTANCE_THRESHOLD;
    }

    public boolean isCoralRight() {
        return rightLaserDistance <= Constants.GroundIntakeConstants.CORAL_RIGHT_DISTANCE_THRESHOLD;
    }

    /**
     * Checks if coral is present in the intake based on current draw
     * @return true if coral is detected
     */
    public boolean isCoralLoaded() {
        coralInRange = CANrange.getIsDetected().getValue();
        if (coralInRange) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * This method is called automatically by the SubsystemNetworkManager
     */
    @Override
    public void updateNetwork() {
        coralLoadedNT.set(isCoralLoaded());
        rightIntakeSetpointNT.set(currentState.getRightSpeed());
        leftIntakeSetpointNT.set(currentState.getLeftSpeed());
        midIntakeSetpointNT.set(currentState.getMidSpeed());
        rightIntakeVelocityNT.set(intakeRight.signals().velocity().getValueAsDouble());
        leftIntakeVelocityNT.set(intakeLeft.signals().velocity().getValueAsDouble());
        midIntakeVelocityNT.set(intakeMid.signals().velocity().getValueAsDouble());
        coralLoadedNT.set(isCoralLoaded());
    }

    @Override
    public void initializeNetwork() {
        // Network initialization if needed
    }
}