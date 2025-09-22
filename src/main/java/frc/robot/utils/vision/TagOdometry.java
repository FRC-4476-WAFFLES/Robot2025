// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.vision;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.VisionConstants;

/** Add your docs here. */
public class TagOdometry {
    public record TagPoseEstimate(
        Pose2d pose,
        double timestampSeconds,
        Matrix<N3, N1> standardDeviation,
        int numTags,
        Pose2d odometryAtTimestamp
    ) {}

    /** Networktables */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable softwareTable = inst.getTable("SoftwareInfo");
    private final StructPublisher<Pose2d> validPoseNT = softwareTable.getStructTopic("Validated Pose", Pose2d.struct).publish();

    /** Limelight hardware */
    public final LimelightContainer leftLimelight = new LimelightContainer(VisionConstants.LIMELIGHT_NAME_L);
    public final LimelightContainer rightLimelight = new LimelightContainer(VisionConstants.LIMELIGHT_NAME_R);

    public void update() {
        // Throttle performance while disabled to prevent overheating
        if (DriverStation.isEnabled()) {
            leftLimelight.setEnabled();
            rightLimelight.setEnabled();
        } else {
            leftLimelight.setDisabled();
            rightLimelight.setDisabled();
        }

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
            chosenEstimate = combineEstimates(leftEstimate.get(), rightEstimate.get());
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

    private Optional<TagPoseEstimate> combineEstimates(TagPoseEstimate a, TagPoseEstimate b) {
        // Ensure A is the most recent pose
        if (a.timestampSeconds < b.timestampSeconds) {
            var tmp = a;
            a = b;
            b = tmp;   
        }

        // Latency compensate the older pose to match the more recent one's timestamp
        Transform2d b_T_a =
            a.odometryAtTimestamp
            .minus(b.odometryAtTimestamp);

        Pose2d poseA = a.pose;
        Pose2d poseB = b.pose.transformBy(b_T_a);

        // Perform inverse variance weighting
        return Optional.of(
            new TagPoseEstimate(
                pose, 
                0, 
                null, 
                0
            )
        );
    }

    /**
     * Checks if both limelights see a tag, used for pit debugging
     * @return true if both limelights see a tag
     */
    public boolean limelightsSeeTag() {
        return leftLimelight.canSeeTag() && rightLimelight.canSeeTag();
    }
}
