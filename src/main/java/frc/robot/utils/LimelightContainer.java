// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.data.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

/** Encapsulates the logic for megatag based localization with a megatag */
public class LimelightContainer {
    private String limelightName;
    private DriveSubsystem driveSubsystem;

    public LimelightContainer(String name, DriveSubsystem subsystem) {
        limelightName = name;
        driveSubsystem = subsystem;
    }
    
    /**
     * Call every periodic loop to update odometry with vision reported poses
     */
    public void update() {
        // Update valid tag IDs, done periodically since they may change on the fly later
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, VisionHelpers.getValidTagIDs());

        //System.out.println(getPigeon2().getAngularVelocityZWorld().getValueAsDouble());

        // Integrate position from mt2
        LimelightHelpers.PoseEstimate mt2Result = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (mt2Result != null) {
            if(mt2Result.tagCount > 0) //Math.abs(Math.toDegrees(getCurrentRobotChassisSpeeds().omegaRadiansPerSecond)) < 10 &&
            {
                driveSubsystem.addVisionMeasurement(
                    mt2Result.pose,
                    Utils.fpgaToCurrentTime(mt2Result.timestampSeconds),
                    VisionHelpers.getEstimationStdDevsLimelightMT2(mt2Result.rawFiducials));

                SmartDashboard.putNumberArray(limelightName + " Pose MT2 ", new double[] {
                    mt2Result.pose.getX(),
                    mt2Result.pose.getY(),
                    mt2Result.pose.getRotation().getDegrees()
                });
            }
        }
        // Integrate rotation from mt1
        LimelightHelpers.PoseEstimate mt1Result = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (mt1Result != null) {
            
            if(mt1Result.tagCount > 0) 
            {
                // Only accept in enabled if close to existing pose
                // If disabled ignore distance heuristic
                // if (getRobotPose().getTranslation().getDistance(mt1Result.pose.getTranslation()) < VisionConstants.MT1_REJECT_DISTANCE || DriverStation.isDisabled()) {

                driveSubsystem.addVisionMeasurement(
                    mt1Result.pose,
                    Utils.fpgaToCurrentTime(mt1Result.timestampSeconds),
                    VisionHelpers.getEstimationStdDevsLimelight(mt1Result.pose, mt1Result.rawFiducials));

                SmartDashboard.putNumberArray(limelightName + " Pose MT1", new double[] {
                    mt1Result.pose.getX(),
                    mt1Result.pose.getY(),
                    mt1Result.pose.getRotation().getDegrees()
                });
                // }
            }
        }

        // Fuse in angle to limelight
        if (DriverStation.isEnabled()) {
            if (driveSubsystem.notRotating()) {
                onSeeding();
            } else {
                onMoving(); // Use IMU mode 2 while rotating to avoid latency issues
            }

            LimelightHelpers.SetRobotOrientation(limelightName, driveSubsystem.getRobotPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        } else {
            onSeeding();
            // Seeding in disabled (Uses IMU mode 1)
            LimelightHelpers.SetRobotOrientation(limelightName, driveSubsystem.getRobotPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        }
    }

    /**
     * Returns true if the camera can see a tag
     * @return A boolean
     */
    public boolean canSeeTag() {
        return LimelightHelpers.getTV(limelightName);
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
}
