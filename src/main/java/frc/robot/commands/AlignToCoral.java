package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.data.Constants.VisionConstants;
import frc.robot.utils.vision.LimelightHelpers;

public class AlignToCoral extends Command {
  private static final String LIMELIGHT_KEY = VisionConstants.LIMELIGHT_NAME_CORAL;

  private SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();
  private SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric();

  private final DoubleSupplier xVelocitySupplier;
  private final DoubleSupplier yVelocitySupplier;
  private final Supplier<Rotation2d> thetaVelocitySupplier;
  private RobotCentric request;
  private Alliance alliance;
  private double latestTx;
  private boolean hasTarget = false;

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
    hasTarget = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightHelpers.getTV(LIMELIGHT_KEY)) {
      latestTx = LimelightHelpers.getTX(LIMELIGHT_KEY);
      hasTarget = true;
    } else {
      if (targetLost.getAsBoolean()) {
        hasTarget = false;
      }
    }

    SmartDashboard.putBoolean("HasTarget Smoothed", hasTarget);

    if (
      !RobotContainer.groundSuperstructure.anyCoralSensorActive() && 
      !RobotContainer.intakeSubsystem.manipulatorLoaded() && 
      hasTarget && 
      LimelightHelpers.getTA(LIMELIGHT_KEY) > 0.75
    ) {  
      if (yVelocitySupplier == null) {
          // if we don't supply a y velocity, move forward at a set speed and align with the note
          if (hasTarget) {
              request = createSwerveRequest(2.0, -0.05 * LimelightHelpers.getTX(LIMELIGHT_KEY));
          } else {
              request = createSwerveRequest(0, 0);
          }
      } else if(RobotContainer.groundSuperstructure.isIntaking()) {
          // if we supply a y velocity, and the intake is running in, align with the note and move forward at the set speed  
          double translationFieldOrientedAngle = Math.atan2(yVelocitySupplier.getAsDouble(), xVelocitySupplier.getAsDouble());
          Rotation2d angleDifference = RobotContainer.driveSubsystem.getRobotPose().getRotation().minus(new Rotation2d(translationFieldOrientedAngle));
          // Calculate the dot product
          double dotProduct = angleDifference.getCos() * Math.hypot(yVelocitySupplier.getAsDouble(), xVelocitySupplier.getAsDouble());
          // Scale the Limelight TX adjustment based on the magnitude of the dot product
          double scaleFactor = Math.abs(dotProduct) / 2.8;
        //   SmartDashboard.putNumber("scaleFactor", scaleFactor);
          double scaledTXAdjustment = -0.05 * latestTx * scaleFactor;
          request = createSwerveRequest(dotProduct, scaledTXAdjustment);
      }
      RobotContainer.driveSubsystem.setControl(request);
    } else if (!DriverStation.isAutonomous()) {
      double speedDeadband = PhysicalConstants.maxSpeed * 0.05;
      double rotationDeadband = PhysicalConstants.maxAngularSpeed * 0.01;

      // if we do have a note, don't apply any note alignment
      RobotContainer.driveSubsystem.setControl(
        driveRequest
          .withDeadband(speedDeadband)
          .withRotationalDeadband(rotationDeadband)
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo)
          .withVelocityX(xVelocitySupplier.getAsDouble())
          .withVelocityY(yVelocitySupplier.getAsDouble())
          .withRotationalRate(thetaVelocitySupplier.get().getRadians())
      );
    }
    else if (DriverStation.isAutonomous() && !hasTarget) {
      // if we are in autonomous and don't have a target, stop the robot
      RobotContainer.driveSubsystem.setControl(new SwerveRequest.Idle());
    }
  }

  // Helper to create SwerveRequest
  private RobotCentric createSwerveRequest(double velocityX, double velocityY) {
    return robotCentricRequest
      .withDeadband(PhysicalConstants.maxSpeed * 0.03)
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo)
      .withVelocityX(-velocityX)
      .withVelocityY(-velocityY);
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
      if (RobotContainer.groundSuperstructure.isHandoffReady()) {
        return true;
      }

      double robotX = RobotContainer.driveSubsystem.getRobotPose().getX();
  
      // Unwrap the Optional and return false if no alliance is available (or handle in some other way)
      if (alliance == null || alliance != DriverStation.getAlliance().orElseThrow(() -> new IllegalStateException("Alliance not set"))) {
          alliance = DriverStation.getAlliance().orElseThrow(() -> new IllegalStateException("Alliance not set"));
      }
  
      // End command if autonomous and the robot is driving to the other side of the field and could get a penalty  
      return DriverStation.isAutonomous() &&
            ((alliance == Alliance.Red && robotX < 8.2) || (alliance == Alliance.Blue && robotX > 8.5));
  }
}
