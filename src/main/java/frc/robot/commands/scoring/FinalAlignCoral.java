// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.PhysicalConstants;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;


import static frc.robot.RobotContainer.*;

public class FinalAlignCoral extends Command {
  /* PID Controllers */
  private PIDController xPidController = new PIDController(3.8, 0, 0.1);
  private PIDController yPidController = new PIDController(3.8, 0, 0.1);
  private PIDController thetaPidController = new PIDController(5.0, 0, 0.1);

  private SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

  private Pose2d targetPose2d;

  private static final double PosMaxError = 0.02;
  private static final double RotMaxError = 1; // degrees

  /* Timing variables */
  private final Timer alignmentTimer = new Timer();
  private static final NetworkTable scoringTable = NetworkTableInstance.getDefault().getTable("ScoringMetrics");
  private static final DoublePublisher alignmentTimePublisher = scoringTable.getDoubleTopic("FinalAlignCoral Duration").publish();


  /** 
   * Command that drives the robot to align with coral scoring
   */
  public FinalAlignCoral(Pose2d targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

    thetaPidController.enableContinuousInput(-Math.PI, Math.PI);

    targetPose2d = targetPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("TerminalCoralAlignment", true);
    
    // Start the alignment timer
    alignmentTimer.reset();
    alignmentTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedDeadband = PhysicalConstants.maxSpeed * 0.01;
    double rotationDeadband = PhysicalConstants.maxAngularSpeed * 0.003;

    int sign = 1;
    if (Math.abs(RobotContainer.driveSubsystem.getOperatorForwardDirection().getDegrees()) > 90) {
      // probably fipped controls lmao
      sign = -1;
    }

    var currentPose = driveSubsystem.getRobotPose();
    double xVelocity = (sign * xPidController.calculate(currentPose.getX(), targetPose2d.getX()));
    double yVelocity = (sign * yPidController.calculate(currentPose.getY(), targetPose2d.getY()));
    double thetaVelocity = thetaPidController.calculate(currentPose.getRotation().getRadians(), targetPose2d.getRotation().getRadians());

    driveSubsystem.setControl(
      driveRequest
        .withDeadband(speedDeadband)
        .withRotationalDeadband(rotationDeadband)
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withVelocityX(xVelocity)
        .withVelocityY(yVelocity)
        .withRotationalRate(thetaVelocity)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("TerminalCoralAlignment", false);

    // Stop the timer and publish the final alignment time
    alignmentTimer.stop();
    double finalAlignmentTime = alignmentTimer.get();
    alignmentTimePublisher.set(finalAlignmentTime);
    
    // Log the alignment time to SmartDashboard as well
    SmartDashboard.putNumber("Recent Alignment Time", finalAlignmentTime);

    driveSubsystem.setControl(
      driveRequest
        .withDeadband(0)
        .withRotationalDeadband(0)
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(0)
    );
  }

  public static boolean isWithinAllowableScoringRange(Pose2d target) {
    var currentPose = driveSubsystem.getRobotPose();
    return currentPose.getTranslation().getDistance(target.getTranslation()) <= PosMaxError &&
      Math.abs(currentPose.getRotation().minus(target.getRotation()).getDegrees()) < RotMaxError;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isWithinAllowableScoringRange(targetPose2d);
  }
}
