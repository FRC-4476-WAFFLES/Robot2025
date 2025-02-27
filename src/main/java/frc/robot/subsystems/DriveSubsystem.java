package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.VisionConstants;
import frc.robot.data.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.utils.LimelightContainer;
import frc.robot.utils.VisionHelpers;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class DriveSubsystem extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* Vision */
    // Since odometry is already handled in the drive subsystem, might as well handle vision here too
    // private Vision visionLeft = new Vision(frc.robot.data.VisionConstants.kCameraLeft,
    //     frc.robot.data.VisionConstants.kRobotToLeftCamera);
    // private Vision visionRight = new Vision(frc.robot.data.VisionConstants.kCameraRight,
    //     frc.robot.data.VisionConstants.kRobotToRightCamera);
    private final LimelightContainer leftLimelight = new LimelightContainer(VisionConstants.LIMELIGHT_NAME_L, this);
    private final LimelightContainer rightLimelight = new LimelightContainer(VisionConstants.LIMELIGHT_NAME_R, this);

    /* Swerve request used for autos */
    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity);

    /* Information about the physical capabilities of the drivetrain, used by Pathplanner */
    public RobotConfig PathPlannerConfig;
    
    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineRotation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public DriveSubsystem(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);

        if (Utils.isSimulation()) {
            startSimThread();
        }

        setup();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public DriveSubsystem(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);

        if (Utils.isSimulation()) {
            startSimThread();
        }

        setup();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public DriveSubsystem(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        
        if (Utils.isSimulation()) {
            startSimThread();
        }

        setup();
    }

    /**
     * Called by all constructor variations.
    */
    private void setup() {
        configurePathPlanner();
        // Sets IMU mode on limelight
        leftLimelight.onSeeding();
        rightLimelight.onSeeding();
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        updateVisionOdometry();
    }

    /**
     * Adds camera reported poses from apriltags to the subsystem's odometry
     */
    private void updateVisionOdometry() {
        if (!isOdometryValid()) {
            return;
        }

        
        leftLimelight.update();
        rightLimelight.update();



        // Commented out sicne we don't have a camera setup figured out yet
        // var visionEstimationLeft = visionLeft.getEstimatedGlobalPose();
        // var visionEstimationRight = visionRight.getEstimatedGlobalPose();

        // if (visionEstimationLeft.isPresent() && visionEstimationRight.isPresent()) {
        //     var estLeft = visionEstimationLeft.get();
        //     Pose2d estPoseLeft = estLeft.estimatedPose.toPose2d();
        //     Matrix<N3, N1> estStdDevsLeft = visionLeft.getEstimationStdDevs(estPoseLeft);

        //     var estRight = visionEstimationRight.get();
        //     Pose2d estPoseRight = estRight.estimatedPose.toPose2d();
        //     Matrix<N3, N1> estStdDevsRight = visionRight.getEstimationStdDevs(estPoseRight);

        //     // Determine which camera has a better estimate based on total variance
        //     double totalVarianceLeft = Math.pow(estStdDevsLeft.get(0, 0), 2) + Math.pow(estStdDevsLeft.get(1, 0), 2) + Math.pow(estStdDevsLeft.get(2, 0), 2);
        //     double totalVarianceRight = Math.pow(estStdDevsRight.get(0, 0), 2) + Math.pow(estStdDevsRight.get(1, 0), 2) + Math.pow(estStdDevsRight.get(2, 0), 2);

        //     Pose2d selectedPose;
        //     double selectedTimestamp;
        //     Matrix<N3, N1> selectedStdDevs;

        //     if (totalVarianceLeft < totalVarianceRight) {
        //         selectedPose = estPoseLeft;
        //         selectedTimestamp = estLeft.timestampSeconds;
        //         selectedStdDevs = estStdDevsLeft;
        //     } else {
        //         selectedPose = estPoseRight;
        //         selectedTimestamp = estRight.timestampSeconds;
        //         selectedStdDevs = estStdDevsRight;
        //     }

        //     SmartDashboard.putNumberArray("Camera Pose", new double[] {
        //         selectedPose.getX(),
        //         selectedPose.getY(),
        //         selectedPose.getRotation().getDegrees()
        //     });

        //     addVisionMeasurement(selectedPose, Utils.fpgaToCurrentTime(selectedTimestamp), selectedStdDevs);
        // } else {
        //     var presentEstimation = visionEstimationLeft.isPresent() ? visionEstimationLeft : visionEstimationRight;
        //     if (presentEstimation.isPresent()) {
        //         var est = presentEstimation.get();
        //         Pose2d estPose = est.estimatedPose.toPose2d();

                

        //         Matrix<N3, N1> estStdDevs = visionEstimationLeft.isPresent()
        //                 ? visionLeft.getEstimationStdDevs(estPose)
        //                 : visionRight.getEstimationStdDevs(estPose);

                
        //         SmartDashboard.putNumberArray("Camera Pose", new double[] {
        //             estPose.getX(),
        //             estPose.getY(),
        //             estPose.getRotation().getDegrees()
        //         });

        //         addVisionMeasurement(estPose, Utils.fpgaToCurrentTime(est.timestampSeconds), estStdDevs);
        //     }
        // }
    }

    /**
     * Setup PathPlanner
     */
    private void configurePathPlanner() {
        try {
            PathPlannerConfig = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getRobotPose, // Supplier of current robot pose
                this::resetPose, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds, feedforwards) -> setControl(autoRequest.withSpeeds(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ), // Consumer of ChassisSpeeds to drive the robot
                new PPHolonomicDriveController(
                    new PIDConstants(4.7, 0, 0),
                    new PIDConstants(3.8, 0, 0)
                ),
                PathPlannerConfig,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Add this subsystem to requirements
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
        }
    }

    /**
     * Checks if the robot is not moving.
     * @return true if the robot is stationary, false otherwise.
    */
    public boolean notMoving() {
        ChassisSpeeds speeds = getCurrentRobotChassisSpeeds();
        return Math.abs(speeds.vxMetersPerSecond) < 0.05
                && Math.abs(speeds.vyMetersPerSecond) < 0.05
                && Math.abs(speeds.omegaRadiansPerSecond) < 0.1;
    }

    /**
     * Checks if the robot is not rotating.
     * @return true if the robot is not rotating, false otherwise.
    */
    public boolean notRotating() {
        return Math.abs(getCurrentRobotChassisSpeeds().omegaRadiansPerSecond) < 0.1;
    }

    /**
     * Checks if the robot is moving slowly. (Sub 1 m/s, not normalized)
     * @return true if the robot is moving slowly, false otherwise.
    */
    public boolean slowMoving() {
        ChassisSpeeds speeds = getCurrentRobotChassisSpeeds();
        return Math.abs(speeds.vxMetersPerSecond) < 1
                && Math.abs(speeds.vyMetersPerSecond) < 1
                && Math.abs(speeds.omegaRadiansPerSecond) < 1;
    }

    /**
     * Checks if both limelights see a tag, used for pit debugging
     * @return true if both limelights see a tag
     */
    public boolean limelightsSeeTag() {
        return leftLimelight.canSeeTag() && rightLimelight.canSeeTag();
    }

    /**
     * Gets the current field-relative pose of the robot according to odometry.
     * @return The current robot pose.
     */
    public Pose2d getRobotPose() {
        return this.getState().Pose;
    }

    /**
     * Gets the current chassis speeds of the robot.
     * @return The current chassis speeds.
    */
    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return getState().Speeds;
    }


    /* Simulation */

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
