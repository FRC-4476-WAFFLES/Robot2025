package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controls;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.semiauto.ScoreCoral;
import frc.robot.data.Constants.ScoringConstants.ScoringLevel;
import frc.robot.utils.WafflesUtilities;

/**
 * All units are meters unless otherwise stated. The origin is on the blue alliance side, and coordinates are returned relative to the blue alliance
*/
public class DynamicPathingSubsystem extends SubsystemBase {
    /* Various distances */
    public static final double REEF_MIN_SCORING_DISTANCE = 2.5;
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
    public static final double REEF_PIPE_CENTER_OFFSET = Units.inchesToMeters(6.8); // Fudged
    // Offset from the edge of the reef to score from 
    public static final double REEF_SCORING_POSITION_OFFSET = 0.464;

    /* Human player station physical parameters */
    public static final Translation2d HUMAN_PLAYER_STATION_LEFT_BLUE = new Translation2d(Units.inchesToMeters(33.51), Units.inchesToMeters(25.80));  
    public static final Translation2d HUMAN_PLAYER_STATION_RIGHT_BLUE = new Translation2d(Units.inchesToMeters(33.51), Units.inchesToMeters(291.20));  
    public static final Rotation2d HUMAN_PLAYER_STATION_LEFT_SCORING_ANGLE = Rotation2d.fromDegrees(126);
    public static final Rotation2d HUMAN_PLAYER_STATION_RIGHT_SCORING_ANGLE = Rotation2d.fromDegrees(234);

    /* Processor physical parameters */
    public static final Translation2d PROCCESSOR_BLUE = new Translation2d(Units.inchesToMeters(235.73), Units.inchesToMeters(0.0));  
    public static final Rotation2d PROCESSOR_SCORING_ANGLE = Rotation2d.fromDegrees(90);

    /* Net physical parameters */
    public static final double NET_LINE_X_BLUE = Units.inchesToMeters(204.0);
    public static final Rotation2d NET_SCORING_ANGLE = Rotation2d.kZero;

    /* Path following parameters */
    public static final double MAX_SPEED = 1.0f;
    public static final double MAX_ACCELERATION = 3.0f;

    /* Angular units are radians per second */
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI;
    public static final double MAX_ANGULAR_ACCELERATION = 4 * Math.PI;
    
    private static final Intake intakeSubsystem = RobotContainer.intakeSubsystem;

    /* Persistent state */
    private boolean coralScoringRightSide = false;
    private boolean isPathing;
    // Only used for coral scoring, level can be determined automatically in every other situation
    private ScoringLevel coralScoringLevel = ScoringLevel.L3;

    enum DynamicPathingSituation {
        NONE, // None of the conditions for other situations are met
        REEF_CORAL, // Scoring coral -> has coral loaded and in range of reef
        REEF_ALGAE, // Picking up algae -> has no coral or algae and is in range of reef
        NET, // Scoring algae in net -> has algae and is in range of net
        PROCESSOR, // Scoring algae in processor -> has algae and is in range of processor
        HUMAN_PICKUP // Picking up coral from human player -> has no coral and in range of human player
    }

    /**
     * Returns the current DynamicPathingSituation
     */
    public static DynamicPathingSituation getDynamicPathingSituation() {
        if (isRobotInRangeOfReefPathing()) {
            if (intakeSubsystem.isCoralLoaded()) {
                return DynamicPathingSituation.REEF_CORAL;
            } else if (!intakeSubsystem.isAlgaeLoaded()) {
                return DynamicPathingSituation.REEF_ALGAE;
            }
        }

        if (isRobotInRangeOfNet() && intakeSubsystem.isAlgaeLoaded()) {
            return DynamicPathingSituation.NET;
        }

        if (isRobotInRangeOfProcessor() && intakeSubsystem.isAlgaeLoaded()) {
            return DynamicPathingSituation.PROCESSOR;
        }

        if (!intakeSubsystem.isCoralLoaded() && isRobotInRangeOfHumanPlayer()) {
            return DynamicPathingSituation.HUMAN_PICKUP;
        }

        return DynamicPathingSituation.NONE;
    }

    /**
     * Is the robot within a certain distance of the reef
     * @return a boolean
     */
    public static boolean isRobotInRangeOfReefPathing() {
        var pose = WafflesUtilities.FlipIfRedAlliance(RobotContainer.driveSubsystem.getRobotPose());
        return (pose.getTranslation().getDistance(REEF_CENTER_BLUE) <= REEF_INRADIUS + REEF_MIN_SCORING_DISTANCE);
    }

    /**
     * Is the robot within a bounding box that allows scoring in the net 
     * @return a boolean
     */
    public static boolean isRobotInRangeOfNet() {
        var pose = WafflesUtilities.FlipIfRedAlliance(RobotContainer.driveSubsystem.getRobotPose());
        return pose.getX() >= NET_MIN_SCORING_X && pose.getX() <= NET_MAX_SCORING_X && pose.getY() >= NET_MIN_SCORING_Y && pose.getY() <= NET_MAX_SCORING_Y;
    }

    /**
     * Is the robot within a certain distance of the processor
     * @return a boolean
     */
    public static boolean isRobotInRangeOfProcessor() {
        var pose = WafflesUtilities.FlipIfRedAlliance(RobotContainer.driveSubsystem.getRobotPose());
        return (pose.getTranslation().getDistance(PROCCESSOR_BLUE) <= PROCCESSOR_MIN_SCORING_DISTANCE);
    }

    /**
     * Is the robot within a certain distance of the human player station
     * @return a boolean
     */
    public static boolean isRobotInRangeOfHumanPlayer() {
        var pose = WafflesUtilities.FlipIfRedAlliance(RobotContainer.driveSubsystem.getRobotPose());
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
        SmartDashboard.putString("Pathing Situation", currentSituation.toString());

        switch (currentSituation) {
            case REEF_CORAL:  { // extra curly brackets to keep scopes seperate
                    Pose2d targetCoralPose = getNearestCoralScoringLocation();
                    SmartDashboard.putNumberArray("TargetPose Reef", new double[] {
                        targetCoralPose.getX(),
                        targetCoralPose.getY(),
                        targetCoralPose.getRotation().getDegrees()
                    });

                    var path = DynamicPathingSubsystem.simplePathToPose(targetCoralPose);
                    if (path.isPresent()){ // If path isn't present, aka we're too close to the target to reasonably path, just give up
                        var pathingCommand = getDynamicPathingWrapperCommand(AutoBuilder.followPath(path.get()));
                        cmd = ScoreCoral.scoreCoralWithPath(pathingCommand);
                    }
                }
                break;

            case REEF_ALGAE: {
                    Pose2d targetAlgaePose = getNearestAlgaePickupLocation();
                    SmartDashboard.putNumberArray("TargetPose Reef", new double[] {
                        targetAlgaePose.getX(),
                        targetAlgaePose.getY(),
                        targetAlgaePose.getRotation().getDegrees()
                    });

                    var path = DynamicPathingSubsystem.simplePathToPose(targetAlgaePose);
                    if (path.isPresent()){ // If path isn't present, aka we're too close to the target to reasonably path, just give up
                        cmd = getDynamicPathingWrapperCommand(AutoBuilder.followPath(path.get()));
                    }
                }
                break;

            case NET: {
                    Rotation2d targetNetRotation = WafflesUtilities.FlipAngleIfRedAlliance(NET_SCORING_ANGLE);
                    double targetNetX = WafflesUtilities.FlipXIfRedAlliance(NET_LINE_X_BLUE); 

                    new DriveTeleop(
                        () -> targetNetX, true, // a PID controller or smth lmao
                        Controls::getDriveY, false,
                        () -> targetNetRotation, true
                    );
                }
                break;

            case PROCESSOR: {
                    Rotation2d targetProcessorRotation = WafflesUtilities.FlipAngleIfRedAlliance(PROCESSOR_SCORING_ANGLE);

                    cmd = new DriveTeleop(
                        Controls::getDriveX, false,
                        Controls::getDriveY, false,
                        () -> targetProcessorRotation, true
                    );
                }
                break;

            case HUMAN_PICKUP: {
                    Rotation2d humanPickupRotation = WafflesUtilities.FlipAngleIfRedAlliance(getHumanPlayerPickupAngle());

                    cmd = new DriveTeleop(
                        Controls::getDriveX, false,
                        Controls::getDriveY, false,
                        () -> humanPickupRotation, true
                    );
                }
                break;
                
            default:
                break;
        }
        
        return cmd;
    }

    /**
     * Sets the side which coral scoring targets
     * @param rightSide if the side is to the right or the left, relative to the driver's point of view
     */
    public void setCoralScoringSide(boolean rightSide) {
        if (coralScoringRightSide != rightSide) {
            coralScoringRightSide = rightSide;
            System.out.println("Setting coral scoring to right side: " + coralScoringRightSide);
            // If we're currently pathing, regenerate the path
            if (isPathing) {
                regenerateCurrentCoralPath();
            }
        }
    }

    /**
     * Regenerates the current path using the new side selection while maintaining smooth motion
     */
    private void regenerateCurrentCoralPath() {
        // Get the new target pose with updated side selection
        Pose2d newTargetPose = getNearestCoralScoringLocation();
        
        // Generate a new path from our current position
        var newPath = simplePathToPose(newTargetPose);
        
        if (newPath.isPresent()) {
            // Create and schedule the new scoring command
            var pathCommand = getDynamicPathingWrapperCommand(AutoBuilder.followPath(newPath.get()));

            // Different from normal pathing command.
            // Since started from outside button based scheduling, letting go of the dynamic pathing button would fail to cancel it
            // .onlyWhile() ensures it can still be canceled by letting go of the button
            Command cmd = ScoreCoral.scoreCoralWithPath(pathCommand).onlyWhile(() -> Controls.dynamicPathingButton.getAsBoolean());
            cmd.schedule();
        }
    }

    /**
     * Sets the desired coral scoring level
     * @param level the desired scoring level
     */
    public void setCoralScoringLevel(ScoringLevel level) {
        coralScoringLevel = level;
        System.out.println("Setting coral scoring sevel to: " + coralScoringLevel);
    }

    /**
     * Gets the coral scoring level set by the operator
     * @return a ScoringLevel enum
     */
    public ScoringLevel getCoralScoringLevel() {
        return coralScoringLevel;
    }

    /**
     * Is the robot actively executing a dynamic pathing command
     * @return a boolean
     */
    public boolean isPathing() {
        return isPathing;
    }

    /**
     * Gets the angle the robot should face to pickup from the nearest human player station. Always returns blue alliance angles.
     * @return
     */
    public Rotation2d getHumanPlayerPickupAngle() {
        Pose2d pose = WafflesUtilities.FlipIfRedAlliance(RobotContainer.driveSubsystem.getRobotPose());
        if (pose.getY() > (FlippingUtil.fieldSizeY / 2)) {
            return HUMAN_PLAYER_STATION_LEFT_SCORING_ANGLE;
        }
        return HUMAN_PLAYER_STATION_RIGHT_SCORING_ANGLE;
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
    public Pose2d getNearestAlgaePickupLocation() {
        return getNearestReefLocationStatic(RobotContainer.driveSubsystem.getRobotPose(), false, true);
    }

    /**
     * Gets coordinates in field space to the nearest coral scoring position or algae pickup point.
     * @param pose The robot's position in field space (meters)
     * @param rightSide Choose the coral scoring position on the right side of the robot, otherwise the left one is chosen. This depends on the robot's forward orientation.
     * @param scoringAlgae If true, rightSide is ignored and we path to the center of the reef face
     * @return The coordinates the robot can score from
     */
    public static Pose2d getNearestReefLocationStatic(Pose2d robotPose, boolean rightSide, boolean scoringAlgae) {
        Pose2d pose = WafflesUtilities.FlipIfRedAlliance(robotPose);

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
        if (!scoringAlgae) {
            // Add in offset for chosen coral scoring position 
            outputTranslation = outputTranslation.plus(scoringPositionOffset);
        }

        // Add in reef center to offset to right origin
        outputTranslation = outputTranslation.plus(REEF_CENTER_BLUE);

        Pose2d outputPose = new Pose2d(outputTranslation, invertedInRadiusAngle);

        // If we flipped the pose at the beginning (aka if we're on the red alliance), flip it back at the end.
        outputPose = WafflesUtilities.FlipIfRedAlliance(outputPose);

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

        double angleBetweenPoses = WafflesUtilities.AngleBetweenPoints(startingPose.getTranslation(), endingPose.getTranslation());

        // Get current chassis speeds for smooth transition
        var currentSpeeds = RobotContainer.driveSubsystem.getCurrentRobotChassisSpeeds();
        double currentVelocity = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        
        // Use consistent constraints regardless of current speed
        PathConstraints constraints = new PathConstraints(
            MAX_SPEED,
            MAX_ACCELERATION,
            MAX_ANGULAR_SPEED,
            MAX_ANGULAR_ACCELERATION
        );

        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(startingPose.getTranslation(), Rotation2d.fromDegrees(angleBetweenPoses)),
            new Pose2d(endingPose.getTranslation(), Rotation2d.fromDegrees(angleBetweenPoses))
        );

        // Create the path using the waypoints above with initial velocity
        // The path will naturally slow down to MAX_SPEED using the consistent constraints
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            new IdealStartingState(currentVelocity, startingPose.getRotation()), // Use current velocity and direction as starting state
            new GoalEndState(0.0, endingPose.getRotation()) // Goal end state with holonomic rotation
        );
        path.preventFlipping = true;

        return Optional.of(path);
    }

    /**
     * Wraps a pathing command with setters for isPathing 
     * @param pathingCommand command to wrap
     * @return the wrapped command
     */
    private Command getDynamicPathingWrapperCommand(Command pathingCommand) {
        return pathingCommand.beforeStarting(() -> {isPathing = true;}).finallyDo(() -> {isPathing = false;});
    }
}
