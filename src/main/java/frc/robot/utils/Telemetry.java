package frc.robot.utils;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    /* Build data */
    private final NetworkTable powerTable = inst.getTable("PowerInfo");
    private final DoublePublisher busVoltage = powerTable.getDoubleTopic("Bus Voltage (Volts)").publish();
    private final DoublePublisher temperature = powerTable.getDoubleTopic("Temperature (Celcius)").publish();
    private final DoublePublisher currentDraw = powerTable.getDoubleTopic("Current Draw (Amps)").publish();
    private final DoublePublisher powerDraw = powerTable.getDoubleTopic("Power Draw (Watts)").publish(); 
    private final DoublePublisher energyUsage = powerTable.getDoubleTopic("Energy Usage (Joules)").publish(); 

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

    /**
     * Construct a telemetry subsystem, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;

        // Publish build info once to networktables
        publishBuildInfo();
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      publishPDHInfo();
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
        busVoltage.set(powerDistributionHub.getVoltage());
        temperature.set(powerDistributionHub.getTemperature());
        currentDraw.set(powerDistributionHub.getTotalCurrent());
        powerDraw.set(powerDistributionHub.getTotalPower());
        energyUsage.set(powerDistributionHub.getTotalEnergy());
    }
}
