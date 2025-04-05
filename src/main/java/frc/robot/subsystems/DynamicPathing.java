package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Controls;
import frc.robot.RobotContainer;
import frc.robot.commands.AlignToPose;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.intake.AlgaeOutake;
import frc.robot.commands.intake.CoralIntake;
import frc.robot.commands.scoring.PickupAlgae;
import frc.robot.commands.scoring.ScoreCoral;
import frc.robot.commands.scoring.ScoreNet;
import frc.robot.commands.superstructure.ApplyScoringSetpoint;
import frc.robot.data.Constants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.data.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.data.Constants.ManipulatorConstants.PivotPosition;
import frc.robot.data.Constants.ScoringConstants.ScoringLevel;
import frc.robot.utils.WafflesUtilities;

/**
 * All units are meters unless otherwise stated. The origin is on the blue alliance side, and coordinates are returned relative to the blue alliance
*/
public class DynamicPathing extends SubsystemBase {
    /* Various distances */
    public static final double REEF_MIN_SCORING_DISTANCE = 2.7;
    public static final double REEF_MAX_SCORING_DISTANCE = 0.3; // Don't try to score within this distance
    public static final double REEF_MIN_SCORING_DISTANCE_L1 = 0.78; // Don't try to score within this distance
    public static final double PROCCESSOR_MIN_SCORING_DISTANCE = 2.2;
    public static final double HUMAN_PLAYER_MIN_PICKUP_DISTANCE = 2;

    /* Net AABB bounds */
    public static final double NET_MIN_SCORING_X = Units.inchesToMeters(150);
    public static final double NET_MIN_SCORING_Y = Units.inchesToMeters(158.50);

    public static final double NET_MAX_SCORING_X = Units.inchesToMeters(340);
    public static final double NET_MAX_SCORING_Y = Units.inchesToMeters(317);

    /* Reef physical parameters */
    public static final Translation2d REEF_CENTER_BLUE = new Translation2d(Units.inchesToMeters(176.745), Units.inchesToMeters(158.50)); 
    public static final double REEF_INRADIUS = 0.81901;
    public static final double REEF_PIPE_CENTER_OFFSET = Units.inchesToMeters(6.5); // Fudged

    /* Various robot to reef distances */
    /* Robot is assumed to have bumpers on */
    public static final double REEF_SCORING_POSITION_OFFSET_ALGAE_CLEARANCE = PhysicalConstants.withBumperBotHalfWidth + 0.65; 
    public static final double REEF_SCORING_POSITION_OFFSET = PhysicalConstants.withBumperBotHalfWidth + 0.12; 
    public static final double REEF_SCORING_POSITION_OFFSET_L1 = PhysicalConstants.withBumperBotHalfWidth + 0.37; 
    public static final double REEF_SCORING_POSITION_OFFSET_L4 = PhysicalConstants.withBumperBotHalfWidth + 0.015; 
    public static final double REEF_PICKUP_POSITION_OFFSET_ALGAE = PhysicalConstants.withBumperBotHalfWidth + 0.015; 
    public static final double REEF_ALGAE_SAFETY_DISTANCE = PhysicalConstants.withBumperBotHalfWidth + 0.35;
    public static final double REEF_ELEVATOR_RETRACTION_DISTANCE = PhysicalConstants.withBumperBotHalfWidth + 0.24;
    public static final double L4_ELEVATOR_DEPLOY_DISTANCE = PhysicalConstants.withBumperBotHalfWidth + 1.1;
    public static final double REEF_L1_HEADING_LOCK_DISTANCE = PhysicalConstants.withBumperBotHalfWidth + 1.0;
    /* Coral scoring pathing parameters */
    public static final double REEF_PATH_POSITION_OFFSET = 0.12; // Distance from reef to handover from path to PID
    public static final double CORAL_PATH_END_SPEED = 0.8; // m/s

    /* Human player station physical parameters */
    public static final Translation2d HUMAN_PLAYER_STATION_RIGHT_BLUE = new Translation2d(Units.inchesToMeters(33.51), Units.inchesToMeters(25.80));  
    public static final Translation2d HUMAN_PLAYER_STATION_LEFT_BLUE = new Translation2d(Units.inchesToMeters(33.51), Units.inchesToMeters(291.20));  
    public static final Rotation2d HUMAN_PLAYER_STATION_LEFT_SCORING_ANGLE = Rotation2d.fromDegrees(126).rotateBy(Rotation2d.k180deg); //126
    public static final Rotation2d HUMAN_PLAYER_STATION_RIGHT_SCORING_ANGLE = Rotation2d.fromDegrees(234).rotateBy(Rotation2d.k180deg);

    /* Processor physical parameters */
    public static final Translation2d PROCCESSOR_BLUE = new Translation2d(Units.inchesToMeters(235.73), Units.inchesToMeters(0.0));  
    public static final Rotation2d PROCESSOR_SCORING_ANGLE = Rotation2d.fromDegrees(-90);
    public static final double PROCESSOR_SCORING_DISTANCE_Y = 0.35; // Distance from processor Y in meters to score from 

    /* Net physical parameters */
    public static final double NET_LINE_X_BLUE = 7.80; // Meters
    public static final Rotation2d NET_SCORING_ANGLE = Rotation2d.k180deg;

    /* Path following parameters */
    public static final double MAX_SPEED = 2.5f;
    public static final double MAX_ACCELERATION = 5.0f;

    /* Angular units are radians per second */
    public static final double MAX_ANGULAR_SPEED = 1 * Math.PI;
    public static final double MAX_ANGULAR_ACCELERATION = 2 * Math.PI;
    
    private static final Intake intakeSubsystem = RobotContainer.intakeSubsystem;

    /* Persistent state */
    private boolean coralScoringRightSide = false;
    private boolean isPathing = false; // Is moving
    private Rotation2d currentClosestReefAngle = Rotation2d.kZero;

    // Only used for coral scoring, level can be determined automatically in every other situation
    private ScoringLevel coralScoringLevel = ScoringLevel.L3;
    private DynamicPathingSituation currentPathingSituation = DynamicPathingSituation.NONE;
    
    private boolean isRunningAction = false;
    public Trigger runningAction = new Trigger(() -> isRunningAction); // If a dynamic action command is active
    public Trigger notRunningAction = runningAction.negate(); // If no dynamic action command is active

    public enum DynamicPathingSituation {
        NONE, // None of the conditions for other situations are met
        REEF_CORAL, // Scoring coral -> has coral loaded and in range of reef
        REEF_ALGAE, // Picking up algae -> has no coral or algae and is in range of reef
        NET, // Scoring algae in net -> has algae and is in range of net
        PROCESSOR, // Scoring algae in processor -> has algae and is in range of processor
        HUMAN_PICKUP // Picking up coral from human player -> has no coral and in range of human player
    }

    @Override
    public void periodic() {
        currentPathingSituation = getDynamicPathingSituation();
        currentClosestReefAngle = calculateClosestFaceAngle(RobotContainer.driveSubsystem.getRobotPose());
        SmartDashboard.putString("Pathing Situation", currentPathingSituation.toString());
    }

    /**
     * Returns the current DynamicPathingSituation
     */
    private static DynamicPathingSituation getDynamicPathingSituation() {
        if (!intakeSubsystem.isCoralLoaded() && !intakeSubsystem.isAlgaeLoaded() && isRobotInRangeOfHumanPlayer()) {
            return DynamicPathingSituation.HUMAN_PICKUP;
        }

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

        return DynamicPathingSituation.NONE;
    }

    /**
     * Is the robot within a certain distance of the reef
     * @return a boolean
     */
    public static boolean isRobotInRangeOfReefPathing() {
        var pose = WafflesUtilities.FlipIfRedAlliance(RobotContainer.driveSubsystem.getRobotPose());
        return (
            pose.getTranslation().getDistance(REEF_CENTER_BLUE) <= REEF_INRADIUS + REEF_MIN_SCORING_DISTANCE &&
            pose.getTranslation().getDistance(REEF_CENTER_BLUE) >= REEF_INRADIUS + REEF_MAX_SCORING_DISTANCE
        );
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
     * Gets robot distance to the reef
     * @return a boolean
     */
    public static double getDistanceToReef() {
        var pose = WafflesUtilities.FlipIfRedAlliance(RobotContainer.driveSubsystem.getRobotPose());
        
        return pose.getTranslation().getDistance(REEF_CENTER_BLUE);
    }

    /**
     * Is the robot within a certain distance of the reef
     * @return a boolean
     */
    public static boolean isPastAlgaeClearancePoint() {
        return getDistanceToReef() > REEF_ALGAE_SAFETY_DISTANCE + REEF_INRADIUS;
    }

    /**
     * Is the robot within a certain distance of the reef
     * @return a boolean
     */
    public static boolean isRobotInRangeOfReefL1() {
        return getDistanceToReef() < REEF_L1_HEADING_LOCK_DISTANCE + REEF_INRADIUS;
    }

    /**
     * Is the robot past a certain distance from the reef
     * @return a boolean
     */
    public static boolean isElevatorRetractionSafe() {
        return getDistanceToReef() > REEF_ELEVATOR_RETRACTION_DISTANCE + REEF_INRADIUS;
    }
    

    public static boolean isElevatorL4Ready() {
        return getDistanceToReef() < L4_ELEVATOR_DEPLOY_DISTANCE + REEF_INRADIUS;
    }

    /**
     * Returns a command that runs the current dynamic action, chosen by the getDynamicPathingSituation() function
     * @return A command that performs the current action
     */
    public Command getCurrentDynamicActionCommand() {
        Command cmd = null; // Command that performs the action
        
        // DynamicPathingSituation currentSituation = getDynamicPathingSituation();
        // SmartDashboard.putString("Pathing Situation", currentSituation.toString());
        // lastPathingSituation = currentSituation;

        switch (currentPathingSituation) {
            case REEF_CORAL:  { // extra curly brackets to keep scopes seperate
                    cmd = createCoralScoreCommand();
                }
                break;

            case REEF_ALGAE: {
                    // Use the extracted helper method to create the command
                    cmd = createAlgaePickupCommand();
                }
                break;

            case NET: {
                    Rotation2d targetNetRotation = WafflesUtilities.FlipAngleIfRedAlliance(NET_SCORING_ANGLE);
                    double targetNetX = WafflesUtilities.FlipXIfRedAlliance(NET_LINE_X_BLUE); 

                    cmd = ScoreNet.getScoreNetCommand(targetNetX, targetNetRotation, true);
                }
                break;

            case PROCESSOR: {
                    Rotation2d targetProcessorRotation = WafflesUtilities.FlipAngleIfRedAlliance(PROCESSOR_SCORING_ANGLE);
                    double targetProcessorY = WafflesUtilities.FlipYIfRedAlliance(PROCCESSOR_BLUE.getY() + PROCESSOR_SCORING_DISTANCE_Y + PhysicalConstants.withBumperBotHalfWidth);
                    Double targetProcessorX = WafflesUtilities.FlipXIfRedAlliance(PROCCESSOR_BLUE.getX());
                    Pose2d processorScoringPose = new Pose2d(targetProcessorX, targetProcessorY, targetProcessorRotation);

                    cmd = new ParallelCommandGroup(
                        new AlignToPose(processorScoringPose),
                        new ApplyScoringSetpoint(ScoringLevel.PROCESSOR),
                        new AlgaeOutake()
                    ).finallyDo(() -> {
                        RobotContainer.elevatorSubsystem.setElevatorSetpoint(ElevatorLevel.REST_POSITION);
                        RobotContainer.pivotSubsystem.setPivotPosition(PivotPosition.CLEARANCE_POSITION);
                    });
                    
                }
                break;

            case HUMAN_PICKUP: {
                    Rotation2d humanPickupRotation = WafflesUtilities.FlipAngleIfRedAlliance(getHumanPlayerPickupAngle());

                    cmd = new ParallelDeadlineGroup(
                        new CoralIntake(),
                        new DriveTeleop(
                            Controls::getDriveY, false,
                            Controls::getDriveX, false,
                            () -> humanPickupRotation, true
                        ),
                        new ApplyScoringSetpoint(ScoringLevel.CORAL_INTAKE)
                    ).finallyDo(() -> {
                        if (!RobotContainer.intakeSubsystem.isCoralLoaded()) {
                            // Keep running intake for 5 seconds after ending if no coral detected
                            Command runAfterCommand = new CoralIntake().withTimeout(4);

                            // This is one of the few cases where directly scheduling a command is okay, 
                            // since we don't want it to be canceled by releasing the driver assist button
                            runAfterCommand.schedule();
                        }
                    });

                }
                break;
                
            default:
                break;
        }
        
        if (cmd == null) {
            cmd = new InstantCommand(); // Do nothing fallback in case something goes wrong
        } else {
            cmd = wrapActionStateCommand(cmd);
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
            //System.out.println("Setting coral scoring to right side: " + coralScoringRightSide);
            // If we're currently pathing, regenerate the path
            if (isRunningAction && currentPathingSituation == DynamicPathingSituation.REEF_CORAL) {
                regenerateCurrentCoralPath();
            }
        }
    }

    /**
     * Regenerates the current path using the new side selection while maintaining smooth motion
     */
    private void regenerateCurrentCoralPath() {
        Command cmd = createCoralScoreCommand();

        if (cmd != null) {
            // Different from normal pathing command.
            // Since started from outside button based scheduling, letting go of the dynamic pathing button would fail to cancel it
            // .onlyWhile() ensures it can still be canceled by letting go of the button
            cmd = cmd.onlyWhile(() -> Controls.dynamicPathingButton.getAsBoolean());
            wrapActionStateCommand(cmd).schedule();
        }
    }

    /**
     * Sets the desired coral scoring level
     * @param level the desired scoring level
     */
    public void setCoralScoringLevel(ScoringLevel level) {
        if (level == coralScoringLevel) {
            return;
        }

        // If switching to L1 from something else, or from L1 to something else while pathing, regenerate
        if (isPathing && currentPathingSituation == DynamicPathingSituation.REEF_CORAL && (level == ScoringLevel.L1 || coralScoringLevel == ScoringLevel.L1)) {
            regenerateCurrentCoralPath();
            //System.out.println("Regenerating path to go to L1");
        }

        coralScoringLevel = level;
        //System.out.println("Setting coral scoring sevel to: " + coralScoringLevel);
    }

    /**
     * Gets the coral scoring level set by the operator
     * @return a ScoringLevel enum
     */
    public ScoringLevel getCoralScoringLevel() {
        return coralScoringLevel;
    }

    /**
     * Gets the coral scoring side set by the operator
     * @return if scoring on the right side 
     */
    public boolean getCoralScoringSide() {
        return coralScoringRightSide;
    }

    /**
     * Gets the current dynamic pathing situation
     * @return a DynamicPathingSituation enum
     */
    public DynamicPathingSituation getCurrentPathingSituation() {
        return currentPathingSituation;
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
        if (coralScoringLevel == ScoringLevel.L1) {
            return getNearestReefLocationStatic(RobotContainer.driveSubsystem.getRobotPose(), coralScoringRightSide, true, REEF_SCORING_POSITION_OFFSET_L1);
        }

        if (coralScoringLevel == ScoringLevel.L4) {
            return getNearestReefLocationStatic(RobotContainer.driveSubsystem.getRobotPose(), coralScoringRightSide, false, REEF_SCORING_POSITION_OFFSET_L4);
        }
        return getNearestReefLocationStatic(RobotContainer.driveSubsystem.getRobotPose(), coralScoringRightSide, false, REEF_SCORING_POSITION_OFFSET);
    }

    public Pose2d getCoralLocationOffset(double offset) {
        return getNearestReefLocationStatic(RobotContainer.driveSubsystem.getRobotPose(), coralScoringRightSide, false, offset);
    }

    /**
     * Gets coordinates in field space to the nearest algae scoring position.
     * @return The coordinates the robot can pickup from
     */
    public Pose2d getNearestAlgaePickupLocation() {
        return getNearestReefLocationStatic(RobotContainer.driveSubsystem.getRobotPose(), false, true, REEF_PICKUP_POSITION_OFFSET_ALGAE);
    }

    /**
     * Gets coordinates in field space to the nearest algae clearance position
     * @return The coordinates 
     */
    public Pose2d getNearestAlgaeClearanceLocation() {
        return getNearestReefLocationStatic(RobotContainer.driveSubsystem.getRobotPose(), false, true, REEF_SCORING_POSITION_OFFSET_ALGAE_CLEARANCE);
    }

    /**
     * Gets coordinates in field space to the nearest coral scoring position or algae pickup point.
     * @param pose The robot's position in field space (meters)
     * @param rightSide Choose the coral scoring position on the right side of the robot, otherwise the left one is chosen. This depends on the robot's forward orientation.
     * @param scoringAlgae If true, rightSide is ignored and we path to the center of the reef face
     * @return The coordinates the robot can score from
     */
    public static Pose2d getNearestReefLocationStatic(Pose2d robotPose, boolean rightSide, boolean scoringAlgae, double offsetFromReef) {
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
        Translation2d outputTranslation = inRadiusNormalized.times(REEF_INRADIUS + offsetFromReef);

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
     * Gets the height of the algea on a given side of the reef, based on robot pose 
     * @param robotPose the field centric pose of the robot
     * @return A ScoringLevel for the height of the algea target
     */
    public static ScoringLevel getAlgeaScoringLevel(Pose2d robotPose) {
        Pose2d pose = WafflesUtilities.FlipIfRedAlliance(robotPose);


        Translation2d TranslatedPose = pose.getTranslation().minus(REEF_CENTER_BLUE);
        double angle = Math.toDegrees(Math.atan2(TranslatedPose.getY(), TranslatedPose.getX())) + 30;
        if (angle < 0) {
            angle = 360 + angle;
        }

        int hexagonFace = (int)(angle / 60.0f);
        
        if (hexagonFace % 2 == 0) {
            return ScoringLevel.ALGAE_L1;
        } else {
            return ScoringLevel.ALGAE_L2;
        }
    }

    /**
     * Calculates and the direction to face the nearest reef face
     * @param robotPose the field centric pose of the robot
     * @return A Rotation2d representing robot rotation
     */
    public Rotation2d calculateClosestFaceAngle(Pose2d robotPose) {
        Pose2d pose = WafflesUtilities.FlipIfRedAlliance(robotPose);


        Translation2d TranslatedPose = pose.getTranslation().minus(REEF_CENTER_BLUE);
        double angle = Math.toDegrees(Math.atan2(TranslatedPose.getY(), TranslatedPose.getX())) + 30;
        if (angle < 0) {
            angle = 360 + angle;
        }

        int inRadiusAngle = (int)(angle / 60.0f) * 60;
        
        return WafflesUtilities.FlipAngleIfRedAlliance(Rotation2d.fromDegrees(inRadiusAngle));
    }

    /**
     * Gets and the direction to face the nearest reef face
     * @return A Rotation2d representing robot rotation
     */
    public Rotation2d getClosestFaceAngle() {
        return currentClosestReefAngle;
    }

    /**
     * Creates a simple path for PathPlanner which moves the robot to a point.
     * Assumes starting at the robot's current position
     * @param endingPose The target end pose
     * @return The PathPlannerPath
     */
    public static Optional<PathPlannerPath> simplePathToPose(Pose2d endingPose) {
        Pose2d startingPose = RobotContainer.driveSubsystem.getRobotPose();
        
        return generateComplexPath(startingPose, null, endingPose);
    }

    /**
     * Creates a path for PathPlanner which moves the robot to a point, passing through an optional set of other points
     * @param startingPose The starting pose
     * @param endingPose The target end pose
     * @param intermediatePoses An array of positions to travel through
     * @return The PathPlannerPath
     */
    public static Optional<PathPlannerPath> generateComplexPath(Pose2d startingPose, Translation2d[] intermediatePoses, Pose2d endingPose) {
        return generateComplexPath(startingPose, intermediatePoses, endingPose, 0);
    }

    /**
     * Creates a path for PathPlanner which moves the robot to a point, passing through an optional set of other points
     * @param startingPose The starting pose
     * @param endingPose The target end pose
     * @param intermediatePoses An array of positions to travel through
     * @return The PathPlannerPath
     */
    public static Optional<PathPlannerPath> generateComplexPath(Pose2d startingPose, Translation2d[] intermediatePoses, Pose2d endingPose, double endingVelocity) {
        // If distance to target is less than some epsilon (1/2 cm in this case), don't even try.
        if (startingPose.getTranslation().getDistance(endingPose.getTranslation()) < 0.005) {
            return Optional.empty();
        }
        
        // Second pose may change depending on if intermediate poses are supplied
        Translation2d secondPose = endingPose.getTranslation();
        if (intermediatePoses != null) {
            secondPose = intermediatePoses[0];
        }

        List<Pose2d> fullPoseSet = new ArrayList<Pose2d>();

        // If coming at speed create curved path to allow for deceleration
        ChassisSpeeds currentSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            RobotContainer.driveSubsystem.getRobotChassisSpeeds(),
            startingPose.getRotation()
        );

        // Add start and end pose
        fullPoseSet.add(new Pose2d(startingPose.getTranslation(), getOptimalPathHeading(startingPose.getTranslation(), secondPose, currentSpeeds)));
        fullPoseSet.add(new Pose2d(endingPose.getTranslation(), endingPose.getRotation()));
    
        // Insert interediate poses if needed
        if (intermediatePoses != null) {
            List<Pose2d> intermediateWaypoints = new ArrayList<>();

            int i = 0;
            for (Translation2d pose : intermediatePoses) {
                Translation2d nextPose;

                if (i == (intermediatePoses.length - 1)) {
                    nextPose = endingPose.getTranslation();
                } else {
                    nextPose = intermediatePoses[i + 1];
                }

                Rotation2d rot = WafflesUtilities.AngleBetweenPoints(pose, nextPose); 
                intermediateWaypoints.add(new Pose2d(pose, rot));
                
                i++;
            }

            fullPoseSet.addAll(1, intermediateWaypoints);
        }
        
        // A list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        // Generate path waypoints
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            fullPoseSet
        );
        
        // Generate constraints
        double velocityMagnitude = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        // If moving faster than max speed, make max speed current speed + some epsilon
        double maxSpeed = velocityMagnitude > MAX_SPEED ? velocityMagnitude + 0.1 : MAX_SPEED;

        PathConstraints constraints = new PathConstraints(
            maxSpeed,
            MAX_ACCELERATION,
            MAX_ANGULAR_SPEED,
            MAX_ANGULAR_ACCELERATION
        );

        // Create the path using the waypoints above with initial velocity
        // The path will naturally slow down to MAX_SPEED using the consistent constraints
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null,
            new GoalEndState(endingVelocity, endingPose.getRotation()) // Goal end state with holonomic rotation
        );
        path.preventFlipping = true;

        return Optional.of(path);
    }

    /**
     * Gets a direction of travel for pathing to point, taking into account velocity
     * If traveling at speed, do not travel in straight line to target and curve out of it
     * 
     * @return a Rotation2d
     */
    private static Rotation2d getOptimalPathHeading(Translation2d start, Translation2d target, ChassisSpeeds speeds) {
        Rotation2d straightAngle = WafflesUtilities.AngleBetweenPoints(start, target);
        double velocityMagnitude = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

        // If too close or slow enough, use a straight path to avoid shaking
        if (velocityMagnitude < 0.05 || start.getDistance(target) < 1.25) {
            // Not moving, use linear direction
            return straightAngle;
        }
        
        // If driving away from target, make initial heading inverted velocity
        Rotation2d speedDirection = new Rotation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        if (Math.abs(speedDirection.minus(straightAngle).getDegrees()) > 90) {
            return speedDirection.plus(Rotation2d.k180deg);
        }

        return speedDirection;
    }   

    /**
     * Wraps a pathing command with setters for isPathing 
     * @param pathingCommand command to wrap
     * @return the wrapped command
     */
    public Command wrapPathingCommand(Command pathingCommand) {
        return pathingCommand.beforeStarting(() -> {isPathing = true;}).finallyDo(() -> {isPathing = false;});
    }

    /**
     * Wraps an action command with setters for the current action state 
     * @param command command to wrap
     * @return the wrapped command
     */
    public Command wrapActionStateCommand(Command command) {
        return command.beforeStarting(() -> {isRunningAction = true;}).finallyDo(() -> {isRunningAction = false;});
    }

    /**
     * Creates an algae pickup command using the current robot position and the nearest algae pickup location.
     * @return A command to pick up algae, or null if not possible
     */
    public Command createAlgaePickupCommand() {
        Pose2d startingPose = RobotContainer.driveSubsystem.getRobotPose();
        Pose2d targetAlgaePose = getNearestAlgaePickupLocation();
        Pose2d clearancePose = getNearestAlgaeClearanceLocation();

        // Publish telemetry
        SmartDashboard.putNumberArray("TargetPose Reef", new double[] {
            targetAlgaePose.getX(),
            targetAlgaePose.getY(),
            targetAlgaePose.getRotation().getDegrees()
        });

        SmartDashboard.putNumberArray("TargetPose Algae Clearance", new double[] {
            clearancePose.getX(),
            clearancePose.getY(),
            clearancePose.getRotation().getDegrees()
        });

        // Generate paths
        Command arrivalPathingCommand;
        if (isPastAlgaeClearancePoint()) {
            // Make one smooth path in
            var pickupPath = DynamicPathing.generateComplexPath(startingPose, 
                new Translation2d[] {clearancePose.getTranslation()}, 
                targetAlgaePose);

            // Give up if path doesn't exist
            if (pickupPath.isPresent()) {
                arrivalPathingCommand = AutoBuilder.followPath(pickupPath.get());
            } else {
                return null;
            }
        } else {
            // Make two disjointed paths to avoid spline funniness
            var initialBackOffPath = DynamicPathing.simplePathToPose(clearancePose);
            // var arrivalPath = DynamicPathing.generateComplexPath(clearancePose, null, targetAlgaePose);

            // Give up if path doesn't exist
            if (initialBackOffPath.isPresent()) {
                arrivalPathingCommand = Commands.sequence(
                    AutoBuilder.followPath(initialBackOffPath.get()),
                    new AlignToPose(targetAlgaePose)
                );
            } else {
                return null;
            }
        }
        
        var backOffPath = DynamicPathing.generateComplexPath(targetAlgaePose, null, clearancePose, 0.2);
        
        if (backOffPath.isPresent()) {
            // Generate final back off path
            var backoffPathingCommand = AutoBuilder.followPath(backOffPath.get());
            
            return PickupAlgae.pickupAlgaeWithPath(
                arrivalPathingCommand, 
                getAlgeaScoringLevel(startingPose), 
                backoffPathingCommand,
                targetAlgaePose
            );
        }
        
        return null;
    }

    /**
     * Creates an algae pickup command using the current robot position and the nearest algae pickup location.
     * @return A command to pick up algae, or null if not possible
     */
    public Command createCoralScoreCommand() {
        Pose2d startingPose = RobotContainer.driveSubsystem.getRobotPose();
        Pose2d targetCoralPose = getNearestCoralScoringLocation();
        ChassisSpeeds speeds = RobotContainer.driveSubsystem.getRobotChassisSpeeds(); // Robot relative, just needed for magnitude

        // Telemetry
        SmartDashboard.putNumberArray("TargetPose Reef", new double[] {
            targetCoralPose.getX(),
            targetCoralPose.getY(),
            targetCoralPose.getRotation().getDegrees()
        });

        Pose2d debugFlippedPose = FlippingUtil.flipFieldPose(targetCoralPose);
        SmartDashboard.putNumberArray("TargetPose Reef_Flipped", new double[] {
            debugFlippedPose.getX(),
            debugFlippedPose.getY(),
            debugFlippedPose.getRotation().getDegrees()
        });

        // If too close just use PID
        //startingPose.getTranslation().getDistance(targetCoralPose.getTranslation()) < 0.6
        if (true) {
            return ScoreCoral.scoreCoralWithPathAndAlgae(new InstantCommand(), targetCoralPose, Double.MAX_VALUE);
        }

        // Calculate offset pose to generate pathing command to 
        Rotation2d offsetAngle;
        if (Math.hypot(speeds.vxMetersPerSecond, speeds.vxMetersPerSecond) > 0.05) {
            offsetAngle = targetCoralPose.getRotation().plus(Rotation2d.k180deg);
        } else {
            offsetAngle = WafflesUtilities.AngleBetweenPoints(
                targetCoralPose.getTranslation(), startingPose.getTranslation()
            );
        }

        Translation2d offsetTranslation = targetCoralPose.getTranslation();
        Translation2d offsetVector = new Translation2d(REEF_PATH_POSITION_OFFSET, offsetAngle); 
        offsetTranslation = offsetTranslation.plus(offsetVector);
        Pose2d offsetCoralPose = new Pose2d(offsetTranslation, targetCoralPose.getRotation());

        
        var path = DynamicPathing.generateComplexPath(startingPose, null, offsetCoralPose, CORAL_PATH_END_SPEED);
        if (path.isPresent()){ // If path isn't present, aka we're too close to the target to reasonably path, just give up
            var pathingCommand = AutoBuilder.followPath(path.get());
            return ScoreCoral.scoreCoralWithPathAndAlgae(pathingCommand, targetCoralPose, CORAL_PATH_END_SPEED);
        }
        
        // Return null if cannot path
        return null;
    }
}
