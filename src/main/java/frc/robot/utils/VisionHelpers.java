package frc.robot.utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.data.Constants.VisionConstants;
import frc.robot.utils.LimelightHelpers.RawFiducial;

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
     * The standard deviations of the estimated pose from
     * {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public static Matrix<N3, N1> getEstimationStdDevsLimelight(Pose2d estimatedPose, RawFiducial[] tags) {
        var estStdDevs = VisionConstants.kSingleTagStdDevsMT1;

        int numTags = 0;
        double avgDist = 0;
        double avgAmbiguity = 0;
        for (var tgt : tags) {
            numTags++;
            avgDist += tgt.distToCamera;
            avgAmbiguity += tgt.ambiguity;
        }
        if (numTags == 0)
            return estStdDevs;
        
        avgDist /= numTags;
        avgAmbiguity /= numTags;

        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
            estStdDevs = VisionConstants.kMultiTagStdDevsMT1;
        
        if (avgAmbiguity > 0.7) {
            return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }

        estStdDevs.times((1 + avgAmbiguity) * 5);
        SmartDashboard.putNumber("LL MT1 Ambiguity", avgAmbiguity);
        
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 2.75)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    /**
     * The standard deviations of the estimated pose from
     * {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     */
    public static Matrix<N3, N1> getEstimationStdDevsLimelightMT2(RawFiducial[] tags) {
        var estStdDevs = VisionConstants.kStdDevsMT2ReefTargeting;

        // var pathingSituation = RobotContainer.dynamicPathingSubsystem.getCurrentPathingSituation();
        // if (pathingSituation == DynamicPathingSituation.REEF_ALGAE || pathingSituation == DynamicPathingSituation.REEF_CORAL) {
        //     estStdDevs = kStdDevsMT2ReefTargeting;
        // }
        

        int numTags = 0;
        double avgDist = 0;
        for (var tgt : tags) {
            numTags++;
            avgDist += tgt.distToCamera;
        }
        if (numTags == 0)
            return estStdDevs;
        avgDist /= numTags;

        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
            estStdDevs.times(0.7);

        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 5)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist * 5));

        return estStdDevs;
    }
}
