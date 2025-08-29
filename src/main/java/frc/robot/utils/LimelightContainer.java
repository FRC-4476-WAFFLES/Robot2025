// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Objects;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.data.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;

/** Encapsulates the logic for megatag based localization with a limelight */
public class LimelightContainer {
    private String limelightName;
    private DriveSubsystem driveSubsystem;

    // Check if limelight is connected
    private double lastHeartbeatValue = -1;
    private double lastHeartbeatTime = -1;

    private boolean isAlive = false;

    public LimelightContainer(String name, DriveSubsystem subsystem) {
        this.limelightName = Objects.requireNonNull(name, "Limelight name cannot be null");
        this.driveSubsystem = Objects.requireNonNull(subsystem, "DriveSubsystem cannot be null");
    }
    
    /**
     * Call every periodic loop to update odometry with vision reported poses. 
     * LimelightHelpers.Flush() or equivalent must be called after all limelights have run update() 
     */
    public void update() {
        // Check for limelight heartbeat
        double heartBeat = LimelightHelpers.getLimelightNTDouble(limelightName, "hb");
        if (lastHeartbeatValue != heartBeat) {
            lastHeartbeatValue = heartBeat;
            lastHeartbeatTime = Timer.getFPGATimestamp();
        }
        isAlive = (Timer.getFPGATimestamp() - lastHeartbeatTime) < VisionConstants.LL_HEARTBEAT_MIN_FREQ;

        // Do not integrate last reported pose if limelight disconnected
        if (!isAlive) return;

        // Update valid tag IDs, done periodically since they may change on the fly later
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, VisionHelpers.getValidTagIDs());

        // Early exit if no tags visible to avoid unnecessary processing
        if (!LimelightHelpers.getTV(limelightName)) {
            updateRobotOrientation(); // Still need to update robot orientation
            return;
        }

        // Integrate position from mt2
        LimelightHelpers.PoseEstimate megatag2Result = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (megatag2Result != null && megatag2Result.tagCount > 0) {
                if (!isValidPose(megatag2Result.pose)) {
                    return;
                }

                var standardDeviations = VisionHelpers.getEstimationStdDevsLimelightMT2(megatag2Result.rawFiducials);
                if (standardDeviations != null && standardDeviations.get(0, 0) > 0) {
                    driveSubsystem.addVisionMeasurement(
                        megatag2Result.pose,
                        Utils.fpgaToCurrentTime(megatag2Result.timestampSeconds),
                        standardDeviations);

                    SmartDashboard.putNumberArray(limelightName + " Pose MT2 ", new double[] {
                        megatag2Result.pose.getX(),
                        megatag2Result.pose.getY(),
                        megatag2Result.pose.getRotation().getDegrees()
                    });

                    SmartDashboard.putNumber(limelightName + "STDEV MT2", standardDeviations.get(0, 0));
                }
        }

        // Integrate rotation from mt1
        LimelightHelpers.PoseEstimate megatag1Result = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (megatag1Result != null && megatag1Result.tagCount > 0) {
                if (!isValidPose(megatag1Result.pose)) {
                    return;
                }
                var estimationStdDevs = VisionHelpers.getEstimationStdDevsLimelight(megatag1Result.pose, megatag1Result.rawFiducials);
                if (estimationStdDevs != null) {
                    driveSubsystem.addVisionMeasurement(
                        megatag1Result.pose,
                        Utils.fpgaToCurrentTime(megatag1Result.timestampSeconds),
                        estimationStdDevs);

                    SmartDashboard.putNumberArray(limelightName + " Pose MT1", new double[] {
                        megatag1Result.pose.getX(),
                        megatag1Result.pose.getY(),
                        megatag1Result.pose.getRotation().getDegrees()
                    });
                }
        }

        updateRobotOrientation();
    }
    
    /**
     * Updates robot orientation and IMU mode for the limelight
     */
    private void updateRobotOrientation() {
        // Fuse in angle to limelight
        if (DriverStation.isEnabled()) {
            if (driveSubsystem.notRotating()) {
                onSeeding();
            } else {
                onMoving(); // Use IMU mode 2 while rotating to avoid latency issues
            }

            LimelightHelpers.SetRobotOrientation_NoFlush(limelightName, driveSubsystem.getRobotPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        } else {
            onSeeding();
            // Seeding in disabled (Uses IMU mode 1)
            LimelightHelpers.SetRobotOrientation_NoFlush(limelightName, driveSubsystem.getRobotPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        }
        // SetRobotOrientation_NoFlush() is used since SetRobotOrientation() flushes NT implicitly
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
     * Validates that a pose estimate contains valid (non-NaN) values
     * @param pose The pose to validate
     * @return true if the pose is valid, false otherwise
     */
    private boolean isValidPose(Pose2d pose) {
        if (pose == null) {
            return false;
        }
        
        return !Double.isNaN(pose.getX()) && 
               !Double.isNaN(pose.getY()) &&
               !Double.isNaN(pose.getRotation().getDegrees()) &&
               Double.isFinite(pose.getX()) &&
               Double.isFinite(pose.getY()) &&
               Double.isFinite(pose.getRotation().getDegrees());
    }
}
