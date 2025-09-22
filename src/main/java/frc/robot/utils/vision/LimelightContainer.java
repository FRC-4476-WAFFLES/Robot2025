// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.vision;

import java.util.Objects;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.VisionConstants;
import frc.robot.utils.vision.LimelightHelpers.PoseEstimate;
import frc.robot.utils.vision.LimelightHelpers.RawFiducial;
import frc.robot.utils.vision.TagOdometry.TagPoseEstimate;

/** Encapsulates the logic for megatag based localization with a limelight */
public class LimelightContainer {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private String limelightName;

    // Connection monitoring
    private double lastHeartbeatValue = -1;
    private double lastHeartbeatTime = -1;
    private boolean isAlive = false;

    private final NetworkTable softwareTable = inst.getTable("SoftwareInfo");
    private final NetworkTable limelightTable;
    private final StructPublisher<Pose3d> megatagRawNT;
    private final StructPublisher<Pose2d> megatagAcceptedNT;
    private final StructPublisher<Pose2d> gyroFusedAcceptedNT;
    private final IntegerPublisher tagCountNT;
    private final StringPublisher chosenTypeNT;

    // Timestamp deduplication
    private double lastMT1Timestamp = -1;

    public LimelightContainer(String name) {
        this.limelightName = Objects.requireNonNull(name, "Limelight name cannot be null");
        
        limelightTable = softwareTable.getSubTable(limelightName);
        megatagRawNT = limelightTable.getStructTopic("MT1 Raw", Pose3d.struct).publish();
        tagCountNT = limelightTable.getIntegerTopic("Tag Count Raw").publish();

        megatagAcceptedNT = limelightTable.getStructTopic("Megatag Accepted", Pose2d.struct).publish();
        gyroFusedAcceptedNT = limelightTable.getStructTopic("Gyro Fused", Pose2d.struct).publish();

        chosenTypeNT = limelightTable.getStringTopic("Estimate Type").publish();
    }
    
    /**
     * Call every periodic loop to update odometry with vision reported poses. 
     * LimelightHelpers.Flush() or equivalent must be called after all limelights have run update() 
     */
    public Optional<TagPoseEstimate> update() {
        // Update connection status
        double heartBeat = LimelightHelpers.getLimelightNTDouble(limelightName, "hb");
        if (lastHeartbeatValue != heartBeat) {
            lastHeartbeatValue = heartBeat;
            lastHeartbeatTime = Timer.getFPGATimestamp();
        }
        isAlive = (Timer.getFPGATimestamp() - lastHeartbeatTime) < VisionConstants.LL_HEARTBEAT_MIN_FREQ;

        // Skip if disconnected
        if (!isAlive) return Optional.empty();

        // Update valid tag IDs
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, VisionHelpers.getValidTagIDs());

        // Skip if no tags visible
        if (!LimelightHelpers.getTV(limelightName)) {
            return Optional.empty();
        }

        // Process MegaTag1
        LimelightHelpers.PoseEstimate megatag1Result = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (megatag1Result != null && megatag1Result.tagCount > 0) {
            // Skip duplicates
            if (megatag1Result.timestampSeconds > lastMT1Timestamp) {
                if (isValidPose(megatag1Result.pose)) {
                    // Validate Z-axis
                    Pose3d pose3d = LimelightHelpers.getBotPose3d_wpiBlue(limelightName);
                    if (Math.abs(pose3d.getZ()) <= VisionConstants.MAX_Z_ERROR) {
                        megatagRawNT.set(pose3d);
                        tagCountNT.set(megatag1Result.tagCount);

                        var megatagEstimate = filterMegatagEstimate(megatag1Result);
                        var gyroEstimate = calculateGyroEstimate(megatag1Result);
                              
                        
                        if (megatagEstimate.isPresent()) {
                            megatagAcceptedNT.set(megatagEstimate.get().pose());
                        }
                        if (gyroEstimate.isPresent()) {
                            gyroFusedAcceptedNT.set(gyroEstimate.get().pose());
                        }
                        
                        if (megatagEstimate.isPresent()) {
                            chosenTypeNT.set("MEGATAG");
                            return megatagEstimate;
                        } else if (gyroEstimate.isPresent()) {
                            chosenTypeNT.set("GYRO");
                            return gyroEstimate;
                        }
                        chosenTypeNT.set("NONE");
                    }
                }
            }
        }

        return Optional.empty();
    }

    private Optional<TagPoseEstimate> filterMegatagEstimate(PoseEstimate megatagResult) {
        // Single-tag validation
        if (megatagResult.tagCount == 1) {
            if (!isAmbiguityAcceptable(megatagResult.rawFiducials)) {
                return Optional.empty();
            }

            if (megatagResult.avgTagArea < VisionConstants.MIN_TAG_AREA_SINGLE_TAG) {
                return Optional.empty();
            }
            
            // For small tags, also check yaw difference
            if (megatagResult.avgTagArea < VisionConstants.MIN_TAG_AREA_FOR_YAW_CHECK) {
                if (!isYawDifferenceAcceptable(megatagResult)) {
                    return Optional.empty();   
                }
            }
        }

        // If not disabled, ensure tag is within a certain distance
        // if (!DriverStation.isDisabled()) {
        //     if (
        //         megatagResult.pose.minus(driveSubsystem.getRobotPose()).getTranslation().getNorm() 
        //         < VisionConstants.MEGATAG1_MAX_DISTANCE_THRESHOLD
        //     ) {
        //         return Optional.empty();   
        //     }
        // }

        var estimationStdDevs = VisionHelpers.getEstimationStdDevsMegatag(megatagResult);
        var odometryAtTimestamp = RobotContainer.telemetry.getPoseAtTimestamp(megatagResult.timestampSeconds);

        // Edgecase handling for if pose buffer hasn't been filled yet or the megatagResult is extremely out of date 
        if (odometryAtTimestamp.isEmpty()) {
            return Optional.empty();
        }

        return Optional.of(new TagPoseEstimate(
            megatagResult.pose,
            megatagResult.timestampSeconds,
            estimationStdDevs,
            megatagResult.tagCount,
            odometryAtTimestamp.get()
        ));
    }

    private Optional<TagPoseEstimate> calculateGyroEstimate(PoseEstimate megatagResult) {
        // Prefer megatag 1 when more than one tag is visible
        if (megatagResult.tagCount > 1) {
            return Optional.empty();
        }

        var odometryAtTimestamp = RobotContainer.telemetry.getPoseAtTimestamp(megatagResult.timestampSeconds);
        // Edgecase handling for if pose buffer hasn't been filled yet or the megatagResult is extremely out of date 
        if (odometryAtTimestamp.isEmpty()) {
            return Optional.empty();
        }

        // Filter out estimates taken while spinning too fast (latency compensation has it's limits)
        if (RobotContainer.telemetry.getYawVelocityAtTimestamp(
                megatagResult.timestampSeconds
            ).orElse(Double.POSITIVE_INFINITY) > VisionConstants.MAX_YAW_RATE_RADS) {

            return Optional.empty();
        }

        var tagPose3d = VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(megatagResult.rawFiducials[0].id);
        if (tagPose3d.isEmpty()) {
            return Optional.empty();
        }

        Pose2d tagPose2d = new Pose2d(tagPose3d.get().toPose2d().getTranslation(), Rotation2d.kZero);
        Pose2d robotToTag = tagPose2d.relativeTo(megatagResult.pose);

        Pose2d calculatedPose =
            new Pose2d(
                tagPose2d
                    .getTranslation()
                    .minus(
                        robotToTag
                            .getTranslation()
                            .rotateBy(odometryAtTimestamp.get().getRotation())),
                odometryAtTimestamp.get().getRotation());

        var estimationStdDevs = VisionHelpers.getEstimationStdDevsGyroFusion(megatagResult);

        return Optional.of(new TagPoseEstimate(
            calculatedPose,
            megatagResult.timestampSeconds,
            estimationStdDevs,
            1,
            odometryAtTimestamp.get()
        ));
    }

    /**
     * Returns true if the camera can see a tag
     * @return A boolean
     */
    public boolean canSeeTag() {
        return LimelightHelpers.getTV(limelightName) && isAlive;
    }

    /** 
     * Runs cameras unthrottled while enabled
     */
    public void setEnabled() {
        LimelightHelpers.SetThrottle(limelightName, 0);
    }

    /**
     * Throttles cameras to manage temperature while robot is disabled
     */
    public void setDisabled() {
        LimelightHelpers.SetThrottle(limelightName, VisionConstants.LIMELIGHT_DISABLED_THROTTLE);
    }

    /**
     * Check for limelight heartbeat
     */
    public boolean isAlive() {
        return isAlive;
    }

    /**
     * Return the limelight's name
     * @return string
     */
    public String getName() {
        return limelightName;
    }
    
    /**
     * Validates that a pose estimate contains valid values and is reasonable
     * @param pose The pose to validate
     * @return true if the pose is valid, false otherwise
     */
    private boolean isValidPose(Pose2d pose) {
        if (pose == null) {
            return false;
        }
        
        // Check for NaN/infinite values
        if (Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) ||
            Double.isNaN(pose.getRotation().getDegrees()) ||
            !Double.isFinite(pose.getX()) || !Double.isFinite(pose.getY()) ||
            !Double.isFinite(pose.getRotation().getDegrees())) {
            return false;
        }
        
        // Check if pose is too close to field origin (common vision failure)
        return pose.getTranslation().getNorm() >= VisionConstants.MIN_POSE_DISTANCE_FROM_ORIGIN;
    }

    /**
     * Checks if the ambiguity of detected tags is acceptable
     * @param tags Array of raw fiducial detections
     * @return true if ambiguity is below threshold, false otherwise
     */
    private boolean isAmbiguityAcceptable(RawFiducial[] tags) {
        if (tags == null || tags.length == 0) {
            return false;
        }

        for (RawFiducial tag : tags) {
            if (tag.ambiguity > VisionConstants.AMBIGUITY_THRESHOLD) {
                return false;
            }
        }
        
        return true;
    }

    /**
     * Checks if the yaw difference between vision and odometry is acceptable for small tags
     * @param visionPose The pose estimate from vision
     * @param timestamp the timestamp of the vision estimate for latency compensation
     * @return true if yaw difference is below threshold, false otherwise
     */
    private boolean isYawDifferenceAcceptable(PoseEstimate visionPose) {
        var odometryPose = RobotContainer.telemetry.getPoseAtTimestamp(visionPose.timestampSeconds);
        if (odometryPose.isEmpty()) {
            return false;
        }

        double yawDifference = Math.abs(MathUtil.angleModulus(
            odometryPose.get().getRotation().getRadians() - visionPose.pose.getRotation().getRadians()
        ));
        
        return Math.toDegrees(yawDifference) <= VisionConstants.MAX_YAW_DIFFERENCE_DEG;
    }
}
