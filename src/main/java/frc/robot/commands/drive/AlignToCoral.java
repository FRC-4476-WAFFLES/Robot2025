package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.subsystems.telemetry.CoralTracking;

public class AlignToCoral extends Command {
  
  private static final double MAX_SPEED = 1;
  private static final double AUTO_APPROACH_SPEED = 0.75;

  private RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric();
  private FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric();

  private CoralTracking coralTracking = RobotContainer.telemetry.coralTracking;

  // For use in auto
  public AlignToCoral() {
    addRequirements(RobotContainer.driveSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (
      !RobotContainer.groundSuperstructure.anyCoralSensorActive() && 
      !RobotContainer.intakeSubsystem.manipulatorLoaded() && 
      coralTracking.hasTarget()
    ) {  

      var inputVector = new Translation2d(Controls.getDriveY() , Controls.getDriveX());
      double scaledInput = MathUtil.clamp(inputVector.getNorm(), -MAX_SPEED, MAX_SPEED);
      double approachSpeed = DriverStation.isAutonomous() ? AUTO_APPROACH_SPEED : scaledInput;

      RobotContainer.driveSubsystem.setControl(
        driveRobotRelative(
          -approachSpeed, 
          MathUtil.clamp(coralTracking.getLatestTX() / 20, -MAX_SPEED, MAX_SPEED), 
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