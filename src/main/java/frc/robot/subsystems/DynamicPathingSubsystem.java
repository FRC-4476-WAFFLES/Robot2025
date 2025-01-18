package frc.robot.subsystems;

import java.io.Console;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveTeleop;

/**
 * All units are meters unless otherwise stated. The origin is on the blue alliance side, and coordinates are returned relative to the blue alliance
*/
public class DynamicPathingSubsystem extends SubsystemBase {
    /* Offset from the edge of the reef to score from  */
    public static final double REEF_SCORING_POSITION_OFFSET = 1;

    /* Distances around various points */
    public static final double REEF_MIN_SCORING_DISTANCE = 2;
    public static final double PROCCESSOR_MIN_SCORING_DISTANCE = 1.5;
    public static final double HUMAN_PLAYER_MIN_PICKUP_DISTANCE = 2;

    /* Net AABB bounds */
    public static final double NET_MIN_SCORING_X = Units.inchesToMeters(200);
    public static final double NET_MIN_SCORING_Y = Units.inchesToMeters(158.50);

    public static final double NET_MAX_SCORING_X = Units.inchesToMeters(245);
    public static final double NET_MAX_SCORING_Y = Units.inchesToMeters(317);

    /* Reef physical parameters */
    public static final Translation2d REEF_CENTER_BLUE = new Translation2d(Units.inchesToMeters(176.745), Units.inchesToMeters(158.50)); 
    public static final double REEF_INRADIUS = 0.81901;
    public static final double REEF_PIPE_CENTER_OFFSET = Units.inchesToMeters(6.5);

    /* Human player station physical parameters */
    public static final Translation2d HUMAN_PLAYER_STATION_LEFT_BLUE = new Translation2d(Units.inchesToMeters(33.51), Units.inchesToMeters(25.80));  
    public static final Translation2d HUMAN_PLAYER_STATION_RIGHT_BLUE = new Translation2d(Units.inchesToMeters(33.51), Units.inchesToMeters(291.20));  

    /* Processor physical parameters */
    public static final Translation2d PROCCESSOR_BLUE = new Translation2d(Units.inchesToMeters(235.73), Units.inchesToMeters(0.0));  

    /* Net physical parameters */
    public static final double NET_LINE_X_BLUE = Units.inchesToMeters(204.0);

    /* Path following parameters */
    public static final double MAX_SPEED = 1.0f;
    public static final double MAX_ACCELERATION = 3.0f;

    /* Angular units are radians per second */
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI;
    public static final double MAX_ANGULAR_ACCELERATION = 4 * Math.PI;
    
    private boolean coralScoringRightSide = false;

    enum DynamicPathingSituation {
        NONE, // None of the conditions for other situations are met
        REEF_CORAL, // Scoring coral -> has coral loaded and in range of reef
        REEF_ALGEA, // Picking up algea -> has no coral or algea and is in range of reef
        NET, // Scoring algea in net -> has algea and is in range of net
        PROCESSOR, // Scoring algea in processor -> has algea and is in range of processor
        HUMAN_PICKUP // Picking up coral from human player -> has no coral and in range of human player
    }

    public static DynamicPathingSituation getDynamicPathingSituation() {
        if (isRobotInRangeOfReefPathing()) {
            if (Manipulator.hasCoralLoaded()) {
                return DynamicPathingSituation.REEF_CORAL;
            } else if (!Manipulator.hasAlgeaLoaded()) {
                return DynamicPathingSituation.REEF_ALGEA;
            }
        }

        if (isRobotInRangeOfNet() && Manipulator.hasAlgeaLoaded()) {
            return DynamicPathingSituation.NET;
        }

        if (isRobotInRangeOfProcessor() && Manipulator.hasAlgeaLoaded()) {
            return DynamicPathingSituation.PROCESSOR;
        }

        if (!Manipulator.hasCoralLoaded() && isRobotInRangeOfHumanPlayer()) {
            return DynamicPathingSituation.HUMAN_PICKUP;
        }

        return DynamicPathingSituation.NONE;
    }

        /**
     * Is the robot within a certain distance of the reef
     * @return a boolean
     */
    public static boolean isRobotInRangeOfReefPathing() {
        var pose = FlipIfRedAlliance(RobotContainer.driveSubsystem.getRobotPose());
        return (pose.getTranslation().getDistance(REEF_CENTER_BLUE) <= REEF_INRADIUS + REEF_MIN_SCORING_DISTANCE);
    }

    /**
     * Is the robot within a bounding box that allows scoring in the net 
     * @return a boolean
     */
    public static boolean isRobotInRangeOfNet() {
        var pose = FlipIfRedAlliance(RobotContainer.driveSubsystem.getRobotPose());
        return pose.getX() >= NET_MIN_SCORING_X && pose.getX() <= NET_MAX_SCORING_X && pose.getY() >= NET_MIN_SCORING_Y && pose.getY() <= NET_MAX_SCORING_Y;
    }

    /**
     * Is the robot within a certain distance of the processor
     * @return a boolean
     */
    public static boolean isRobotInRangeOfProcessor() {
        var pose = FlipIfRedAlliance(RobotContainer.driveSubsystem.getRobotPose());
        return (pose.getTranslation().getDistance(PROCCESSOR_BLUE) <= PROCCESSOR_MIN_SCORING_DISTANCE);
    }

    /**
     * Is the robot within a certain distance of the human player station
     * @return a boolean
     */
    public static boolean isRobotInRangeOfHumanPlayer() {
        var pose = FlipIfRedAlliance(RobotContainer.driveSubsystem.getRobotPose());
        boolean inRangeLeft = (pose.getTranslation().getDistance(HUMAN_PLAYER_STATION_LEFT_BLUE) <= HUMAN_PLAYER_MIN_PICKUP_DISTANCE);
        boolean inRangeRight = (pose.getTranslation().getDistance(HUMAN_PLAYER_STATION_RIGHT_BLUE) <= HUMAN_PLAYER_MIN_PICKUP_DISTANCE);
        return inRangeLeft || inRangeRight;
    }

    /**
     * Returns a command that paths to the chosen dynamic path target, chosen by the getDynamicPathingSituation() function
     * @return A pathplanner command that drives to the chosen position
     */
    public Command getCurrentDynamicPathCommand() {
        Command cmd = new InstantCommand(); // Do nothing fallback in case something goes wrong
        
        DynamicPathingSituation currentSituation = getDynamicPathingSituation();
        

        switch (currentSituation) {
            case REEF_CORAL: 
                { // extra curly brackets to keep scopes seperate
                    Pose2d targetCoralPose = getNearestCoralScoringLocation();
                    SmartDashboard.putNumberArray("TargetPose Reef", new double[] {
                        targetCoralPose.getX(),
                        targetCoralPose.getY(),
                        targetCoralPose.getRotation().getDegrees()
                    });

                    DynamicPathingSubsystem.simplePathToPose(targetCoralPose).ifPresent((var path) -> {
                        // If path isn't present, aka we're too close to the target to reasonably path, just give up
                        cmd = AutoBuilder.followPath(path);
                    });
                }
                break;

            case REEF_ALGEA:
                {
                    Pose2d targetAlgeaPose = getNearestAlgeaPickupLocation();
                    SmartDashboard.putNumberArray("TargetPose Reef", new double[] {
                        targetAlgeaPose.getX(),
                        targetAlgeaPose.getY(),
                        targetAlgeaPose.getRotation().getDegrees()
                    });

                    DynamicPathingSubsystem.simplePathToPose(targetAlgeaPose).ifPresent((var path) -> {
                        // If path isn't present, aka we're too close to the target to reasonably path, just give up
                        cmd = AutoBuilder.followPath(path);
                    });
                }
                break;
            case NET:
                new DriveTeleop(
                    // a PID controller or smth lmao
                    Controls::getDriveY,
                    // PID controller lmao
                );
                break;
            case PROCESSOR:
                new DriveTeleop(
                    Controls::getDriveX,
                    Controls::getDriveY,
                    // PID controller lmao
                );
                break;
            case HUMAN_PICKUP:
                new DriveTeleop(
                    Controls::getDriveX,
                    Controls::getDriveY,
                    // PID controller lmao
                );
                break;
            default:
                break;
        }
        
        return cmd;
    }

    /**
     * Returns a command that paths to the nearest coral scoring or algea pickup position
     * @return A pathplanner command that drives to the nearest chosen position
     */
    // public Command getReefPathCommand( ) {
    //     Command cmd = new InstantCommand();

    //     if (isRobotInRangeOfReefPathing()) {
            
            
            
    //     }

    //     return cmd;
    // }

    /**
     * Sets the side which coral scoring targets
     * @param rightSide if the side is to the right or the left, relative to the driver's point of view
     */
    public void setCoralScoringSide(boolean rightSide) {
        coralScoringRightSide = rightSide;
        System.out.println("Setting scoring to right: " + rightSide);
    }

    /**
     * Gets coordinates in field space to the nearest coral scoring position.
     * @return The coordinates the robot can score from
     */
    public Pose2d getNearestCoralScoringLocation() {
        return getNearestReefLocationStatic(RobotContainer.driveSubsystem.getRobotPose(), coralScoringRightSide, false);
    }

    /**
     * Gets coordinates in field space to the nearest coral scoring position.
     * @return The coordinates the robot can score from
     */
    public Pose2d getNearestAlgeaPickupLocation() {
        return getNearestReefLocationStatic(RobotContainer.driveSubsystem.getRobotPose(), false, true);
    }

    /**
     * Gets coordinates in field space to the nearest coral scoring position or algae pickup point.
     * @param pose The robot's position in field space (meters)
     * @param rightSide Choose the coral scoring position on the right side of the robot, otherwise the left one is chosen. This depends on the robot's forward orientation.
     * @param scoringAlgea If true, rightSide is ignored and we path to the center of the reef face
     * @return The coordinates the robot can score from
     */
    public static Pose2d getNearestReefLocationStatic(Pose2d robotPose, boolean rightSide, boolean scoringAlgea) {
        Pose2d pose = FlipIfRedAlliance(robotPose);

        // SmartDashboard.putNumberArray("InProgress Target Pose", new double[] {
        //     pose.getX(),
        //     pose.getY(),
        //     pose.getRotation().getDegrees()
        // });

        Translation2d TranslatedPose = pose.getTranslation().minus(REEF_CENTER_BLUE);
        double angle = Math.toDegrees(Math.atan2(TranslatedPose.getY(), TranslatedPose.getX())) + 30;
        if (angle < 0) {
            angle = 360 + angle;
        }

        

        int inRadiusAngle = (int)(angle / 60.0f) * 60;
        Translation2d inRadiusNormalized = new Translation2d(Math.cos(Math.toRadians(inRadiusAngle)), Math.sin(Math.toRadians(inRadiusAngle)));
        // Offset from center by correct amount to score, also offset out so that robot doesn't intersect reef
        Translation2d outputTranslation = inRadiusNormalized.times(REEF_INRADIUS + REEF_SCORING_POSITION_OFFSET);

        Translation2d scoringPositionOffset = new Translation2d(Math.cos(Math.toRadians(inRadiusAngle + 90)), Math.sin(Math.toRadians(inRadiusAngle + 90)));
        scoringPositionOffset = scoringPositionOffset.times(REEF_PIPE_CENTER_OFFSET);

        Rotation2d invertedInRadiusAngle = Rotation2d.fromDegrees(inRadiusAngle).plus(Rotation2d.k180deg);

        boolean scoringSide = rightSide;
        // Difference between operator forward and inverted inradius angle
        double angleDifference = invertedInRadiusAngle.minus(Rotation2d.kZero).getDegrees();
        if (Math.abs(angleDifference) > 90) {
            scoringSide = !scoringSide;
        }

        // Choose left or right coral scoring position
        scoringPositionOffset = scoringPositionOffset.times(scoringSide ? 1 : -1);

        // Only offset if scoring coral 
        if (!scoringAlgea) {
            // Add in offset for chosen coral scoring position 
            outputTranslation = outputTranslation.plus(scoringPositionOffset);
        }

        // Add in reef center to offset to right origin
        outputTranslation = outputTranslation.plus(REEF_CENTER_BLUE);

        Pose2d outputPose = new Pose2d(outputTranslation, invertedInRadiusAngle);

        // If we flipped the pose at the beginning (aka if we're on the red alliance), flip it back at the end.
        outputPose = FlipIfRedAlliance(outputPose);

        return outputPose;
    }

    /**
     * Creates a simple, straight path for PathPlanner which moves the robot to a point.
     * @param endingPose The target end pose
     * @return The PathPlannerPath
     */
    public static Optional<PathPlannerPath> simplePathToPose(Pose2d endingPose) {
        Pose2d startingPose = RobotContainer.driveSubsystem.getRobotPose();
        
        // If distance to target is less than some epsilon (1/2 cm in this case), don't even try.
        if (startingPose.getTranslation().getDistance(endingPose.getTranslation()) < 0.005) {
            return Optional.empty();
        }

        double angleBetweenPoses = AngleBetweenPoints(startingPose.getTranslation(), endingPose.getTranslation());

        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(startingPose.getTranslation(), Rotation2d.fromDegrees(angleBetweenPoses)),
            new Pose2d(endingPose.getTranslation(), Rotation2d.fromDegrees(angleBetweenPoses))
        );

        PathConstraints constraints = new PathConstraints(MAX_SPEED, MAX_ACCELERATION, MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION); // The constraints for this path.

        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(0.0, endingPose.getRotation()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );
        path.preventFlipping = true;

        
        return Optional.of(path);
    }

    /**
     * Takes a pose and flips it to the other side of the field if the robot is on the red alliance.
     * @param pose The input Pose2d
     * @return The output Pose2d 
     */
    public static Pose2d FlipIfRedAlliance(Pose2d pose) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                return FlippingUtil.flipFieldPose(pose);
            }
        }
        
        return pose;
    }

    /**
     * Takes an angle and flips it to the other side of the field if the robot is on the red alliance.
     * @param pose The input angle (degtees)
     * @return The output angle 
     */
    public static Rotation2d FlipIfRedAllianceAngle(Rotation2d angle) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                return FlippingUtil.flipFieldRotation(angle);
            }
        }
        
        return angle;
    }

    /**
     * Returns the angle from p1 to p2
    */
    public static double AngleBetweenPoints(Translation2d p1, Translation2d p2) {
        return Math.atan2(p2.getY() - p1.getY(), p2.getX() - p1.getX());
    }

    /**
     * Returns the angle for the driver's forward direction, depending on alliance
     * @return a rotation2d representing driver forward in field space
     */
    public static Rotation2d getDriverForwardAngle() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                return Rotation2d.k180deg;
            }
        }
        return Rotation2d.kZero;
    }
}
