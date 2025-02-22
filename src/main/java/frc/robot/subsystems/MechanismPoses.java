package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.ElevatorConstants;
import frc.robot.data.Constants.PhysicalConstants;

public class MechanismPoses extends SubsystemBase {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable mechanismTable = inst.getTable("Mechanism3d");

    // Publishers for each mechanism's pose
    private final StructPublisher<Pose3d> elevatorFirstStagePosePub = mechanismTable.getStructTopic("elevatorFirstStage", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> elevatorCarriagePosePub = mechanismTable.getStructTopic("elevatorCarriage", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> pivotPosePub = mechanismTable.getStructTopic("pivot", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> zeroCalibPosePub = mechanismTable.getStructTopic("zerocalib", Pose3d.struct).publish();

    // Base transforms for each mechanism (relative to robot center)
    private static final Transform3d ELEVATOR_BASE = new Transform3d(
        new Translation3d(0.1208, 0, 0.0699), // Elevator is near the front of the robot
        new Rotation3d(0, 0, 0)
    );

    private static final Transform3d PIVOT_BASE = new Transform3d(
        new Translation3d(0.2858, 0, 0.4128), // Pivot is at the same base as elevator
        new Rotation3d(0, 0, 0)
    );

    // Constants for elevator stages
    private static final double FIRST_STAGE_START_HEIGHT = ElevatorConstants.MAX_ELEVATOR_HEIGHT / 2.0; // Height where first stage starts moving

    public MechanismPoses() {
        // Publish zeroed calibration pose
        zeroCalibPosePub.set(new Pose3d());
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
            carriageHeight = elevatorHeight + 0.0254;
            firstStageHeight = 0;
        } else {
            // First stage starts moving in second half
            double remainingHeight = elevatorHeight - FIRST_STAGE_START_HEIGHT;
            firstStageHeight = remainingHeight;
            carriageHeight = FIRST_STAGE_START_HEIGHT + remainingHeight + 0.0254;
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

        // Publish poses directly
        elevatorFirstStagePosePub.set(firstStagePose);
        elevatorCarriagePosePub.set(carriagePose);
    }

    private void updatePivotPose() {
        double pivotAngle = Math.toRadians(RobotContainer.pivotSubsystem.getPivotPosition());
        double elevatorHeight = RobotContainer.elevatorSubsystem.getElevatorPositionMeters();
        
        // Create pivot pose - rotates around Y axis, moves up with elevator carriage
        Pose3d pivotPose = new Pose3d(
            PIVOT_BASE.getTranslation().plus(new Translation3d(0, 0, elevatorHeight)),
            new Rotation3d(0, pivotAngle, 0).plus(PIVOT_BASE.getRotation())
        );

        // Publish pose directly
        pivotPosePub.set(pivotPose);
    }
} 