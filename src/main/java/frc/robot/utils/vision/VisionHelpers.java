package frc.robot.utils.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.data.Constants.VisionConstants;
import frc.robot.utils.vision.LimelightHelpers.PoseEstimate;
import frc.robot.utils.vision.LimelightHelpers.RawFiducial;

public class VisionHelpers {

    /**
     * Depending on the state of the robot, get which tags to localize off of 
     * @return an int array of valid tag IDs
     */
    public static int[] getValidTagIDs() {
        // Expand to allow other tags while not coral pathing, but for now is ok
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                return VisionConstants.RED_VALID_REEF_TAG_IDs;
            }
        }
        return VisionConstants.BLUE_VALID_REEF_TAG_IDs;
    }

     /**
     * Calculates the standard deviations for a given limelight pose estimate
     * For use with Megatag 1
     */
    public static Matrix<N3, N1> getEstimationStdDevsMegatag(PoseEstimate limelightPoseEstimate) {
        Matrix<N3, N1> estStdDevs;

        if (limelightPoseEstimate.tagCount == 1) {
            estStdDevs = VisionConstants.defaultkSingleTagStdDevsMT1;

            // Reduce the contribution of ambiguous tags
            estStdDevs.times(1 / (1 - limelightPoseEstimate.rawFiducials[0].ambiguity));
        } else {
            // If multiple tags are visible, use different (lower) deviations
            estStdDevs = VisionConstants.defaultMultiTagStdDevsMT1;
        } 

        if (limelightPoseEstimate.tagCount == 0) {
            return estStdDevs;
        }

        // Scale based on distance
        estStdDevs = estStdDevs.times(1 + (limelightPoseEstimate.avgTagDist * limelightPoseEstimate.avgTagDist / 6));

        return estStdDevs;
    }

    /**
     * Calculates the standard deviations for a given limelight pose estimate being fused with gyro orientation
     * For use with Megatag 1 + Gyro estimates
     */
    public static Matrix<N3, N1> getEstimationStdDevsGyroFusion(PoseEstimate limelightPoseEstimate) {
        Matrix<N3, N1> estStdDevs = VisionConstants.defaultStdDevsFusedGyroEstimate;

        if (limelightPoseEstimate.tagCount == 0) {
            return estStdDevs;
        }

        // Lightly scale based on distance
        estStdDevs = estStdDevs.times(1 + (limelightPoseEstimate.avgTagDist * limelightPoseEstimate.avgTagDist / 50));

        return estStdDevs;
    }
}
