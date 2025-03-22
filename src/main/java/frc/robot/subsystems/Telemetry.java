package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.data.BuildConstants;

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
    private final DoublePublisher velocityX = driveStats.getDoubleTopic("Velocity X").publish();
    private final DoublePublisher velocityY = driveStats.getDoubleTopic("Velocity Y").publish();
    private final DoublePublisher speed = driveStats.getDoubleTopic("Speed").publish();
    private final DoublePublisher odomFreq = driveStats.getDoubleTopic("Odometry Frequency").publish();

    /* Build data */
    private final NetworkTable buildTable = inst.getTable("BuildInfo");
    private final StringPublisher deployedBranch = buildTable.getStringTopic("Deployed Branch").publish();
    private final StringPublisher buildTimeStamp = buildTable.getStringTopic("Build Timestamp").publish();
    private final StringPublisher repository = buildTable.getStringTopic("Repository").publish();

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
    private final StringPublisher climbState = controlsTable.getStringTopic("Climb State").publish();
    private final DoublePublisher calculatedDriveX = controlsTable.getDoubleTopic("Calculated Drive X").publish();
    private final DoublePublisher calculatedDriveY = controlsTable.getDoubleTopic("Calculated Drive Y").publish();
    private final DoublePublisher calculatedDriveRot = controlsTable.getDoubleTopic("Calculated Drive Rotation").publish();

    /* Controls data */
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
    
    /*                            */
    /* Swerve Telemetry Variables */
    /*                            */

    /* Keep a reference of the last pose to calculate the speeds */
    private Pose2d m_lastPose = new Pose2d();
    private double lastTime = Utils.getCurrentTimeSeconds();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    private final double MaxSpeed;

    /*                 */
    /* Other Variables */
    /*                 */

    private PowerDistribution powerDistributionHub = new PowerDistribution(1, ModuleType.kRev);

    /* Scoring metrics data */
    private final NetworkTable scoringMetricsTable = inst.getTable("ScoringMetrics");
    
    // Remove tracking of averages, counts, and success rates
    // Only keep publishers for the most recent durations

    /**
     * Construct a telemetry subsystem, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;

        // Publish build info once to networktables
        publishBuildInfo();

        // Set override state once to avoid it sticking around after code reboots
        publishOperatorOverrideInfo();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        publishPDHInfo();

        // Climber telemetry
        if (RobotContainer.currentClimbState != null) {
            climbState.set(RobotContainer.currentClimbState.toString());
        }

        // Pathplanner Telemetry
        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            // pathplannerCurrentPoseNT.set(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            // pathplannerTargetPoseNT.set(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            // pathplannerCurrentTrajectory.set(poses.toArray(trajTypeArray));
        });
    }

    /* Accept the swerve drive state and telemeterize it to smartdashboard */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the pose */
        Pose2d pose = state.Pose;
        fieldTypePub.set("Field2d");
        fieldPub.set(new double[] {
            pose.getX(),
            pose.getY(),
            pose.getRotation().getDegrees()
        });

        /* Telemeterize the robot's general speeds */
        double currentTime = Utils.getCurrentTimeSeconds();
        double diffTime = currentTime - lastTime;
        lastTime = currentTime;
        Translation2d distanceDiff = pose.minus(m_lastPose).getTranslation();
        m_lastPose = pose;

        Translation2d velocities = distanceDiff.div(diffTime);

        speed.set(velocities.getNorm());
        velocityX.set(velocities.getX());
        velocityY.set(velocities.getY());
        odomFreq.set(1.0 / state.OdometryPeriod);

        /* Telemeterize the module's states */
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));

            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }

        driveSpeed.set(state.ModuleStates[0].speedMetersPerSecond);
        anglePos.set(state.ModuleStates[0].angle.getDegrees());
        driveSetpoint.set(state.ModuleTargets[0].speedMetersPerSecond);
        angleSetpoint.set(state.ModuleTargets[0].angle.getDegrees());

        //SignalLogger.writeDoubleArray("odometry", new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
        //SignalLogger.writeDouble("odom period", state.OdometryPeriod, "seconds");
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
}
