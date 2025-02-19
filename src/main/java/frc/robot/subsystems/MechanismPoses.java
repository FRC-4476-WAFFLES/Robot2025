package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.ElevatorConstants;
import frc.robot.data.Constants.PhysicalConstants;

public class MechanismPoses extends SubsystemBase {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable mechanismTable = inst.getTable("Mechanism");

    // Publishers for each mechanism's pose
    private final DoubleArrayPublisher elevatorFirstStagePosePub = mechanismTable.getDoubleArrayTopic("elevatorFirstStagePose").publish();
    private final DoubleArrayPublisher elevatorCarriagePosePub = mechanismTable.getDoubleArrayTopic("elevatorCarriagePose").publish();
    private final DoubleArrayPublisher pivotPosePub = mechanismTable.getDoubleArrayTopic("pivotPose").publish();

    // Base transforms for each mechanism (relative to robot center)
    private static final Transform3d ELEVATOR_BASE = new Transform3d(
        new Translation3d(PhysicalConstants.withBumperBotHalfWidth - 0.1, 0.0, 0.1), // Elevator is near the front of the robot
        new Rotation3d(0, 0, 0)
    );

    private static final Transform3d PIVOT_BASE = new Transform3d(
        new Translation3d(PhysicalConstants.withBumperBotHalfWidth - 0.1, 0.0, 0.1), // Pivot is at the same base as elevator
        new Rotation3d(0, 0, 0)
    );

    // Constants for elevator stages
    private static final double FIRST_STAGE_START_HEIGHT = ElevatorConstants.MAX_ELEVATOR_HEIGHT / 2.0; // Height where first stage starts moving

    public MechanismPoses() {
        // Constructor
    }

    @Override
    public void periodic() {
        updateElevatorPoses();
        updatePivotPose();
    }

    private void updateElevatorPoses() {
        double elevatorHeight = RobotContainer.elevatorSubsystem.getElevatorPositionMeters();
        
        // Calculate stage positions
        double carriageHeight;
        double firstStageHeight;

        if (elevatorHeight <= FIRST_STAGE_START_HEIGHT) {
            // Only carriage moves in first half
            carriageHeight = elevatorHeight;
            firstStageHeight = 0;
        } else {
            // First stage starts moving in second half
            double remainingHeight = elevatorHeight - FIRST_STAGE_START_HEIGHT;
            firstStageHeight = remainingHeight;
            carriageHeight = FIRST_STAGE_START_HEIGHT + remainingHeight;
        }

        // Create first stage pose
        Pose3d firstStagePose = new Pose3d(
            ELEVATOR_BASE.getTranslation().plus(new Translation3d(0, 0, firstStageHeight)),
            ELEVATOR_BASE.getRotation()
        );

        // Create carriage pose
        Pose3d carriagePose = new Pose3d(
            ELEVATOR_BASE.getTranslation().plus(new Translation3d(0, 0, carriageHeight)),
            ELEVATOR_BASE.getRotation()
        );

        publishPose(elevatorFirstStagePosePub, firstStagePose);
        publishPose(elevatorCarriagePosePub, carriagePose);
    }

    private void updatePivotPose() {
        double pivotAngle = Math.toRadians(RobotContainer.pivotSubsystem.getPivotPosition());
        double elevatorHeight = RobotContainer.elevatorSubsystem.getElevatorPositionMeters();
        
        // Create pivot pose - rotates around Y axis, moves up with elevator carriage
        Pose3d pivotPose = new Pose3d(
            PIVOT_BASE.getTranslation().plus(new Translation3d(0, 0, elevatorHeight)),
            new Rotation3d(0, pivotAngle, 0).plus(PIVOT_BASE.getRotation())
        );

        publishPose(pivotPosePub, pivotPose);
    }

    private void publishPose(DoubleArrayPublisher publisher, Pose3d pose) {
        publisher.set(new double[] {
            pose.getX(),
            pose.getY(),
            pose.getZ(),
            pose.getRotation().getX(),
            pose.getRotation().getY(),
            pose.getRotation().getZ()
        });
    }
} 