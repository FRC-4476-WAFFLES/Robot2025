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
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.data.Constants.GroundIntakeConstants;
import frc.robot.utils.NetworkUser;
import frc.robot.utils.PhoenixHelpers;
import frc.robot.utils.SubsystemNetworkManager;
import frc.robot.utils.IO.TalonFXIO;

/**
 * The GroundIntake subsystem handles the robot's L1 intake mechanism.
 * It controls:
 * - An intake motor for collecting game pieces
 */
public class GroundIntake extends SubsystemBase implements NetworkUser{
    // Hardware Components
    private final TalonFXIO intakeLeft;
    private final TalonFXIO intakeRight;
    private final TalonFXIO intakeMid;
    private LaserCan leftLaserCan;
    private LaserCan midLaserCan;
    private LaserCan rightLaserCan;
    private CANrange CANrange = new CANrange(Constants.CANIds.groundIntakeCanRange);
    // Control Objects
    private final MotionMagicVelocityVoltage intakeRightControlRequest = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVelocityVoltage intakeLeftControlRequest = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVelocityVoltage intakeMidControlRequest = new MotionMagicVelocityVoltage(0);
    // State Variables
    public enum GroundIntakeState {
        //TODO make actual states depending on what we want to do
        INTAKE(0, 0,20),
        OUTTAKE(150,0.33,0);
        
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
    private GroundIntakeState currentState = GroundIntakeState.INTAKE;
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
        configureIntakeMotors();
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
    public void periodic() {
        intakeRight.setControl(intakeRightControlRequest.withVelocity(currentState.getRightSpeed()).withSlot(0));
        intakeLeft.setControl(intakeLeftControlRequest.withVelocity(currentState.getLeftSpeed()).withSlot(0));//not sure if they will be following same speed 
        intakeMid.setControl(intakeMidControlRequest.withVelocity(currentState.getMidSpeed()).withSlot(0));
        isCoralLoaded();
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