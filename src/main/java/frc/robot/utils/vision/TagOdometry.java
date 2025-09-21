// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.vision;

import java.util.Optional;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.VisionConstants;
import frc.robot.utils.vision.LimelightHelpers.PoseEstimate;

/** Add your docs here. */
public class TagOdometry {
    public record TagPoseEstimate(
        Pose2d pose,
        double timestampSeconds,
        Matrix<N3, N1> standardDeviation,
        int numTags
    ) {}

    /** Networktables */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable softwareTable = inst.getTable("SoftwareInfo");
    private final StructPublisher<Pose2d> validPoseNT = softwareTable.getStructTopic("Validated Pose", Pose2d.struct).publish();

    /** Limelight hardware */
    public final LimelightContainer leftLimelight = new LimelightContainer(VisionConstants.LIMELIGHT_NAME_L, RobotContainer.driveSubsystem);
    public final LimelightContainer rightLimelight = new LimelightContainer(VisionConstants.LIMELIGHT_NAME_R, RobotContainer.driveSubsystem);

    public void update() {
        // Updates odometry from vision.
        // Does not flush networktables.
        var leftEstimate = leftLimelight.update();
        var rightEstimate = rightLimelight.update();

        // Leftover from when MegaTag2 was in use
        // {
        //     // Flush networktables explicitly once to avoid network latency
        //     // Do not flush once per limelight, since flushing NT is ratelimited to once every 10ms
        //     // With one or more cameras each flushing periodically, you start seeing loop overruns
        //     NetworkTableInstance.getDefault().flush();
        // }

        Optional<TagPoseEstimate> chosenEstimate = Optional.empty();
        if (leftEstimate.isPresent() != rightEstimate.isPresent()) {
            chosenEstimate = leftEstimate.isPresent() ? leftEstimate : rightEstimate;
        } else if (leftEstimate.isPresent() && rightEstimate.isPresent()) {
            chosenEstimate = Optional.of(combineEstimates(leftEstimate.get(), rightEstimate.get()));
        }

        // Only fuse in one estimate to avoid "double tapping" the Kalman filter
        // Prevents excessively weighting vision over odometry
        if (chosenEstimate.isPresent()) {
            var estimate = chosenEstimate.get();
            
            validPoseNT.set(estimate.pose);
            RobotContainer.driveSubsystem.addVisionMeasurement(
                estimate.pose, 
                estimate.timestampSeconds,
                estimate.standardDeviation
            );
        }

        // Provide vision fault warning
        RobotContainer.telemetry.setVisionFault(
            !leftLimelight.isAlive() || !rightLimelight.isAlive()
        );
    }

    private TagPoseEstimate combineEstimates(TagPoseEstimate A, TagPoseEstimate B) {

    }

    /**
     * Checks if both limelights see a tag, used for pit debugging
     * @return true if both limelights see a tag
     */
    public boolean limelightsSeeTag() {
        return leftLimelight.canSeeTag() && rightLimelight.canSeeTag();
    }
}
