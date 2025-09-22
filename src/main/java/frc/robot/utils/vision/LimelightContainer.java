// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.vision;

import java.util.Objects;
import java.util.Optional;

import org.photonvision.PhotonPoseEstimator;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.data.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.vision.LimelightHelpers.PoseEstimate;
import frc.robot.utils.vision.LimelightHelpers.RawFiducial;
import frc.robot.utils.vision.TagOdometry.TagPoseEstimate;

/** Encapsulates the logic for megatag based localization with a limelight */
public class LimelightContainer {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private String limelightName;
    private DriveSubsystem driveSubsystem;

    // Connection monitoring
    private double lastHeartbeatValue = -1;
    private double lastHeartbeatTime = -1;
    private boolean isAlive = false;

    private final NetworkTable softwareTable = inst.getTable("SoftwareInfo");
    private final NetworkTable limelightTable;
    private final StructPublisher<Pose3d> mt1NT;
    private final IntegerPublisher tagCount;
    

    // Timestamp deduplication
    private double lastMT1Timestamp = -1;

    public LimelightContainer(String name, DriveSubsystem subsystem) {
        this.limelightName = Objects.requireNonNull(name, "Limelight name cannot be null");
        this.driveSubsystem = Objects.requireNonNull(subsystem, "DriveSubsystem cannot be null");
        
        limelightTable = softwareTable.getSubTable(limelightName);
        mt1NT = limelightTable.getStructTopic("MT1 Estimate", Pose3d.struct).publish();
        tagCount = limelightTable.getIntegerTopic("tagCount").publish();
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
                        // Single-tag validation
                        boolean passValidation = true;
                        if (megatag1Result.tagCount == 1) {
                            passValidation = isAmbiguityAcceptable(megatag1Result.rawFiducials) &&
                                           megatag1Result.avgTagArea >= VisionConstants.MIN_TAG_AREA;
                            
                            // For small tags, also check yaw difference
                            if (passValidation && megatag1Result.avgTagArea < VisionConstants.MIN_TAG_AREA_FOR_YAW_CHECK) {
                                passValidation = isYawDifferenceAcceptable(megatag1Result.pose);
                            }
                        }
                        
                        if (passValidation) {
                            mt1NT.set(pose3d);
                            tagCount.set(megatag1Result.tagCount);

        // var estimationStdDevs = VisionHelpers.getEstimationStdDevsLimelight(megatag1Result.pose, megatag1Result.rawFiducials);
        // if (estimationStdDevs != null) {
        //     driveSubsystem.addVisionMeasurement(
        //         megatag1Result.pose,
        //         Utils.fpgaToCurrentTime(megatag1Result.timestampSeconds),
        //         estimationStdDevs);
        //     lastMT1Timestamp = megatag1Result.timestampSeconds;
        // }

                            

                            // TODO: Calculate pose, solve based on fused angle
                            return Optional.of(new TagPoseEstimate(
                                pose,
                                heartBeat,
                                null,
                                0
                            ));
                        }
                    }
                }
            }
        }

        return Optional.empty();
    }

    /**
     * Returns true if the camera can see a tag
     * @return A boolean
     */
    public boolean canSeeTag() {
        return LimelightHelpers.getTV(limelightName) && isAlive;
    }

    /**
     * When the robot isn't moving, configure vision mode
     */
    public void onSeeding() {
        LimelightHelpers.SetIMUMode(limelightName, VisionConstants.SEDING_LL_IMU_MODE);
    }

    /**
     * When the robot is moving, configure vision mode
     */
    public void onMoving() {
        LimelightHelpers.SetIMUMode(limelightName, VisionConstants.MOVING_LL_IMU_MODE); 
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
     * @return true if yaw difference is below threshold, false otherwise
     */
    private boolean isYawDifferenceAcceptable(Pose2d visionPose) {
        Pose2d odometryPose = driveSubsystem.getRobotPose();
        
        double yawDifference = Math.abs(MathUtil.angleModulus(
            odometryPose.getRotation().getRadians() - visionPose.getRotation().getRadians()
        ));
        
        return Math.toDegrees(yawDifference) <= VisionConstants.MAX_YAW_DIFFERENCE_DEG;
    }
}
