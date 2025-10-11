package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Controls;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.data.Constants.VisionConstants;
import frc.robot.utils.vision.LimelightHelpers;

public class AlignToCoral extends Command {
  private static final String LIMELIGHT_KEY = VisionConstants.LIMELIGHT_NAME_CORAL;
  private static final double MAX_SPEED = 1;
  private static final double AUTO_APPROACH_SPEED = 0.75;

  private RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric();
  private FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric();

  private final DoubleSupplier xVelocitySupplier;
  private final DoubleSupplier yVelocitySupplier;
  private final Supplier<Rotation2d> thetaVelocitySupplier;

  private double latestTx;
  private double latestTy;
  private boolean hasTarget = false;

  private final NetworkTable softwareTable = NetworkTableInstance.getDefault().getTable("SoftwareInfo");
  private final BooleanPublisher hasTargetNT = softwareTable.getBooleanTopic("Has Target").publish();
  private final DoublePublisher tXNT = softwareTable.getDoubleTopic("Tx").publish();
  private final DoublePublisher tYNT = softwareTable.getDoubleTopic("Ty").publish();
    

  private Trigger targetLost = new Trigger(() -> !LimelightHelpers.getTV(LIMELIGHT_KEY))
    .debounce(0.1);

  /** Creates a new AllignWithNote. */
  public AlignToCoral(DoubleSupplier xVelocitySupplier, DoubleSupplier yVelocitySupplier, Supplier<Rotation2d> thetaVelocitySupplier) {
    addRequirements(RobotContainer.driveSubsystem);
    
    this.xVelocitySupplier = xVelocitySupplier;
    this.yVelocitySupplier = yVelocitySupplier;
    this.thetaVelocitySupplier = thetaVelocitySupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    latestTx = 0;
    latestTy = 100;
    hasTarget = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateCamera();

    if (
      !RobotContainer.groundSuperstructure.anyCoralSensorActive() && 
      !RobotContainer.intakeSubsystem.manipulatorLoaded() && 
      hasTarget
    ) {  

      var inputVector = new Translation2d(Controls.getDriveY() , Controls.getDriveX());
      double scaledInput = MathUtil.clamp(inputVector.getNorm(), -MAX_SPEED, MAX_SPEED);
      double approachSpeed = DriverStation.isAutonomous() ? AUTO_APPROACH_SPEED : scaledInput;

      RobotContainer.driveSubsystem.setControl(
        driveRobotRelative(
          -approachSpeed, 
          MathUtil.clamp(latestTx / 20, -MAX_SPEED, MAX_SPEED), 
          Controls.getDriveRotation().getRadians()
        )
      );
    } else {

      // Drive normally
      RobotContainer.driveSubsystem.setControl(
        driveFieldRelative(
          Controls.getDriveY(),
          Controls.getDriveX(),
          Controls.getDriveRotation().getRadians()
        )
      );
    }
  }

  private void updateCamera() {
    if (LimelightHelpers.getTV(LIMELIGHT_KEY)) {
      if (LimelightHelpers.getTA(LIMELIGHT_KEY) > 0.3) {
        // Reject new targets when our last target was so close it's partly covered by the ground intake
        // [Rejects random flickering to background objects as coral enters intake] 
        if (Math.abs(latestTx) < 15 && Math.abs(latestTy) < 6 && hasTarget) {
          return;
        }
        // Reject switching to radically different targets
        // [If multiple are in frame, pick just one]
        if (Math.abs(LimelightHelpers.getTX(LIMELIGHT_KEY) - latestTx) > 7 && hasTarget) {
          return;
        }


        latestTx = LimelightHelpers.getTX(LIMELIGHT_KEY);
        latestTy = LimelightHelpers.getTY(LIMELIGHT_KEY);
        hasTarget = true;
      }
    } else {
      if (targetLost.getAsBoolean()) {
        hasTarget = false;
      }
    }

    tXNT.set(latestTx);
    tYNT.set(latestTy);
    hasTargetNT.set(hasTarget);
  }

  private RobotCentric driveRobotRelative(double velocityX, double velocityY, double thetaVelocity)  {
    return robotCentricDrive
      .withDeadband(PhysicalConstants.maxSpeed * 0.03)
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo)
      .withVelocityX(velocityX)
      .withVelocityY(velocityY)
      .withRotationalRate(thetaVelocity);
  }

  private FieldCentric driveFieldRelative(double velocityX, double velocityY, double thetaVelocity)  {
    return fieldCentricDrive
      .withDeadband(PhysicalConstants.maxSpeed * 0.03)
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo)
      .withVelocityX(velocityX)
      .withVelocityY(velocityY)
      .withRotationalRate(thetaVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the robot 
    RobotContainer.driveSubsystem.setControl(new SwerveRequest.Idle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.groundSuperstructure.isHandoffReady();
  }
}