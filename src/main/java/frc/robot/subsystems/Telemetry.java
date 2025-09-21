package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.data.BuildConstants;
import frc.robot.data.Constants.CANIds;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.utils.WafflesUtilities;
import frc.robot.utils.IO.DeferredRefresher;
import frc.robot.utils.external.ConcurrentTimeInterpolatableBuffer;

public class Telemetry extends SubsystemBase {
    /*                         */
    /* Networktables Variables */
    /*                         */

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    /* Robot speeds for general checking */
    private final NetworkTable driveStats = inst.getTable("Drive");
    private final DoublePublisher odomFreq = driveStats.getDoubleTopic("Odometry Frequency").publish();

    /* Software metadata */
    private final NetworkTable softwareTable = inst.getTable("SoftwareInfo");
    private final StringPublisher deployedBranch = softwareTable.getStringTopic("Deployed Branch").publish();
    private final StringPublisher buildTimeStamp = softwareTable.getStringTopic("Build Timestamp").publish();
    private final StringPublisher repository = softwareTable.getStringTopic("Repository").publish();

    /* Power data */
    private final NetworkTable powerTable = inst.getTable("PowerInfo");
    private final DoublePublisher busVoltage = powerTable.getDoubleTopic("Bus Voltage (Volts)").publish();
    private final DoublePublisher temperature = powerTable.getDoubleTopic("Temperature (Celcius)").publish();
    private final DoublePublisher currentDraw = powerTable.getDoubleTopic("Current Draw (Amps)").publish();
    private final DoublePublisher powerDraw = powerTable.getDoubleTopic("Power Draw (Watts)").publish(); 
    private final DoublePublisher energyUsage = powerTable.getDoubleTopic("Energy Usage (Joules)").publish(); 

    /* Controls data */
    private final NetworkTable controlsTable = inst.getTable("Controls");
    private final BooleanPublisher overrideEnabled = controlsTable.getBooleanTopic("Override Enabled").publish();
    private final DoublePublisher calculatedDriveX = controlsTable.getDoubleTopic("Calculated Drive X").publish();
    private final DoublePublisher calculatedDriveY = controlsTable.getDoubleTopic("Calculated Drive Y").publish();
    private final DoublePublisher calculatedDriveRot = controlsTable.getDoubleTopic("Calculated Drive Rotation").publish();

    private final DoublePublisher matchTime = controlsTable.getDoubleTopic("Match Time").publish();

    /* Swerve debugging data */
    private final NetworkTable swerveTable = inst.getTable("SwerveModule");
    private final DoublePublisher driveSetpoint = swerveTable.getDoubleTopic("DriveSetpoint").publish();
    private final DoublePublisher driveSpeed = swerveTable.getDoubleTopic("DriveSpeed").publish();
    private final DoublePublisher angleSetpoint = swerveTable.getDoubleTopic("AngleSetpoint").publish();
    private final DoublePublisher anglePos = swerveTable.getDoubleTopic("AnglePos").publish();

    /* Pathplanner data */
    // private final NetworkTable pathplannerTable = inst.getTable("PathPlanner");
    // StructPublisher<Pose2d> pathplannerCurrentPoseNT = pathplannerTable
    //     .getStructTopic("PPCurrentPose", Pose2d.struct).publish();
    // StructPublisher<Pose2d> pathplannerTargetPoseNT = pathplannerTable
    //     .getStructTopic("PPTargetPose", Pose2d.struct).publish();
    // StructArrayPublisher<Pose2d> pathplannerCurrentTrajectory = pathplannerTable
    //     .getStructArrayTopic("PPCurrentTrajectory", Pose2d.struct).publish();

    // private final Pose2d[] trajTypeArray = new Pose2d[0];


    /*                 */
    /* Other Variables */
    /*                 */

    public boolean manipulatorCoralSimLoaded = false;
    public boolean intakeSimLoaded = false;
    public boolean intakeHandoffSimLoaded = false;
    public boolean algeaSimLoaded = false;

    private PowerDistribution powerDistributionHub = new PowerDistribution(1, ModuleType.kRev);

    // CAN checking variables
    private CANStatus rioCanStatus = new CANStatus();

    private Trigger rioCanStatusTrigger = new Trigger(() -> {    
        CANJNI.getCANStatus(rioCanStatus);
        
        return rioCanStatus.receiveErrorCount == 0 && rioCanStatus.transmitErrorCount == 0;
    }).debounce(2); // If error seen in last two seconds, report issue
    
    

    // Async CANivore bus status checking
    private CANBus CANivoreBus = new CANBus(CANIds.CANivoreName);
    DeferredRefresher<CANBusStatus> canivoreRefresher = new DeferredRefresher<>(
        "CANivore Status", 
        CodeConstants.PERIODIC_LOOP_TIME, 
        () -> CANivoreBus.getStatus()
    );
    private Trigger drivetrainCanStatusTrigger = new Trigger(() -> {
        var canStatus = canivoreRefresher.getLatestValue();
        if (canStatus.isPresent()) {
            SmartDashboard.putNumber("ERR", canStatus.get().TEC + canStatus.get().REC);
        
            return (canStatus.get().Status.isOK()
            && canStatus.get().TEC == 0
            && canStatus.get().REC == 0);
        }
        return false; // Error if not present
    }).debounce(2); // If error seen in last two seconds, report issue

    /*                 */
    /*  Alerts System  */
    /*                 */

    private final Alert canFaultDetected = new Alert("CAN fault detected [See Console]", AlertType.kError);
    private final Alert rioCanError = new Alert("RIO CAN bus error", AlertType.kError);
    private final Alert canivoreError = new Alert("CANivore bus error", AlertType.kError);
    private final Alert visionFaultDetected = new Alert("", AlertType.kError);
    // private final Alert driverControllerDisconnected = new Alert("Driver controller disconnected [port 0].", AlertType.kWarning);
    private final Alert leftJoystickDisconnected = new Alert("Left joystick disconnected [port 0].", AlertType.kWarning);
    private final Alert rightJoystickDisconnected = new Alert("Right joystick disconnected [port 1].", AlertType.kWarning);
    private final Alert operatorControllerDisconnected = new Alert("Operator controller disconnected [port 2].", AlertType.kWarning); 

    /*                       */
    /*  Latency Compensation */
    /*                       */

    // Timestamps are in the timebase of Timer.getFPGATimestamp()
    private ConcurrentTimeInterpolatableBuffer<Pose2d> poseHistoryBuffer = 
        ConcurrentTimeInterpolatableBuffer.createBuffer(CodeConstants.POSE_HISTORY_LOOKBACK_TIME);

    /**
     * Construct a telemetry subsystem
     */
    public Telemetry() {
        // MaxSpeed = PhysicalConstants.maxSpeed;

        // Publish build info once to networktables
        publishBuildInfo();

        // Set override state once to avoid it sticking around after code reboots
        publishOperatorOverrideInfo();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        publishPDHInfo();

        matchTime.set(Timer.getMatchTime());

        // Update controls warnings
        // driverControllerDisconnected.set(!Controls.driverController.isConnected());
        leftJoystickDisconnected.set(!Controls.leftJoystick.isConnected());
        rightJoystickDisconnected.set(!Controls.rightJoystick.isConnected());
        operatorControllerDisconnected.set(!Controls.operatorController.isConnected());

        // Check for CAN errors
        rioCanError.set(!rioCanStatusTrigger.getAsBoolean());
        canivoreError.set(!drivetrainCanStatusTrigger.getAsBoolean());
        
        // System.out.println(RobotContainer.canConfigFailed);
        if (Robot.canConfigFailed || !rioCanStatusTrigger.getAsBoolean() || !drivetrainCanStatusTrigger.getAsBoolean()) {
            canFaultDetected.set(true);
        } else {
            canFaultDetected.set(false);
        }
    }

    /* Accept the swerve drive state and post it to networktables */
    public void telemetryConsumer(SwerveDriveState state) {
        Pose2d pose = state.Pose;
        fieldTypePub.set("Field2d");
        fieldPub.set(new double[] {
            pose.getX(),
            pose.getY(),
            pose.getRotation().getDegrees()
        });

        odomFreq.set(1.0 / state.OdometryPeriod);

        /* Debug a module's states */
        // for (int i = 0; i < 4; ++i) {
        //     m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
        //     m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
        //     m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));

        //     SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        // }

        // Log individual module states
        driveSpeed.set(state.ModuleStates[0].speedMetersPerSecond);
        anglePos.set(state.ModuleStates[0].angle.getDegrees());
        driveSetpoint.set(state.ModuleTargets[0].speedMetersPerSecond);
        angleSetpoint.set(state.ModuleTargets[0].angle.getDegrees());

        poseHistoryBuffer.addSample(WafflesUtilities.currentTimeToFPGA(state.Timestamp), pose);
    }

    /**
     * Gets the robot pose at the given timestamp (FPGA timebase) 
     */
    public Optional<Pose2d> getPoseAtTimestamp(double timestamp) {
        return poseHistoryBuffer.getSample(timestamp);
    }

    /**
     * Use autogenerated build constants to display git branch and deployment time
     */
    public void publishBuildInfo() {
        // If you have errors here, it's because the BuildConstants file hasn't been generated. 
        // Run a build and the errors should dissapear
        deployedBranch.set(BuildConstants.GIT_BRANCH);
        buildTimeStamp.set(BuildConstants.BUILD_DATE);
        repository.set(BuildConstants.MAVEN_NAME);
    }

    /**
     * Publish data about power usage to networktables for monitoring
     */
    public void publishPDHInfo() {
        try {
            busVoltage.set(powerDistributionHub.getVoltage());
            temperature.set(powerDistributionHub.getTemperature());
            currentDraw.set(powerDistributionHub.getTotalCurrent());
            powerDraw.set(powerDistributionHub.getTotalPower());
            energyUsage.set(powerDistributionHub.getTotalEnergy());
        } catch (Exception e){
            DriverStation.reportWarning("PDH Firmware Failure.", false);
        }
    }

    /**
     * Publish data about controls
     */
    public void publishControlInfoX(double driveX) {
        calculatedDriveX.set(driveX);    
    }

    /**
     * Publish data about controls
     */
    public void publishControlInfoY(double driveY) {        
        calculatedDriveY.set(driveY);
    }

    
    /**
     * Publish data about controls
     */
    public void publishControlInfoRot(double driveRot) {        
        calculatedDriveRot.set(driveRot);
    }

    /**
     * Update the state of the operator override. Should only be called when changing the state
     */
    public void publishOperatorOverrideInfo() {
        overrideEnabled.set(RobotContainer.isOperatorOverride);
    }

    /**
     * Update the scoring metrics with the latest alignment and scoring durations
     * @param alignmentTime Time it took to complete the alignment in seconds
     * @param totalScoringTime Time it took for the entire scoring operation in seconds
     */
    public void updateScoringMetrics(double alignmentTime, double totalScoringTime) {
        // Only publish to SmartDashboard for driver visibility
        SmartDashboard.putNumber("Recent Alignment Time", alignmentTime);
        SmartDashboard.putNumber("Recent Total Scoring Time", totalScoringTime);
    }
    
    /**
     * Record a scoring operation timing
     * @param alignmentTime Time it took for alignment
     * @param totalTime Total time for the scoring operation
     */
    public void recordScoringTime(double alignmentTime, double totalTime) {
        updateScoringMetrics(alignmentTime, totalTime);
    }

    /**
     * Indicate that there is a Vision fault
     */
    public void setVisionFault(boolean value) {
        visionFaultDetected.set(value);
        if (value) {
            ArrayList<String> details = new ArrayList<>();
            var vision = RobotContainer.driveSubsystem.vision;
            if (!vision.leftLimelight.isAlive()) {
                details.add(vision.leftLimelight.getName());
            }
            if (!vision.rightLimelight.isAlive()) {
                details.add(vision.rightLimelight.getName());
            }

            visionFaultDetected.setText("Vision fault detected [" + String.join(", ", details) + "]");
        }
    }

    public void toggleManipulatorCoralSimLoaded() {
        manipulatorCoralSimLoaded = !manipulatorCoralSimLoaded;
    }

    public void toggleIntakeSimLoaded() {
        intakeSimLoaded = !intakeSimLoaded;
    }

    public void toggleIntakeHandoffSimLoaded() {
        intakeHandoffSimLoaded = !intakeHandoffSimLoaded;
    }

    public void toggleAlgeaSimLoaded() {
        algeaSimLoaded = !algeaSimLoaded;
    }
}
