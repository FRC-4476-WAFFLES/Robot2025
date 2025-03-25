// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.utils.WafflesUtilities;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;


import static frc.robot.RobotContainer.*;

public class AlignToPose extends Command {
  /* PID Controllers */
  private PIDController posPidController = new PIDController(4.1, 0, 0.22);
  private ProfiledPIDController thetaPidController = new ProfiledPIDController(7.0, 0, 0.1, new Constraints(4, 20));

  /* Constants */
  private static final double PosMaxError = 0.02;
  private static final double RotMaxError = 1; // degrees

  /* Instance variables */
  private SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();
  private Pose2d targetPose2d;
  private double maxSpeed;

  /* Timing variables */
  private final Timer alignmentTimer = new Timer();
  private static final NetworkTable scoringTable = NetworkTableInstance.getDefault().getTable("PathingMetrics");
  private static final DoublePublisher alignmentTimePublisher = scoringTable.getDoubleTopic("PID Align Duration").publish();


  /** 
   * Command that drives the robot to align with coral scoring
   */
  public AlignToPose(Pose2d targetPose, double speedLimit) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

    thetaPidController.enableContinuousInput(-Math.PI, Math.PI);

    targetPose2d = targetPose;
    maxSpeed = speedLimit;
  }

  /** 
   * Command that drives the robot to align with coral scoring
   */
  public AlignToPose(Pose2d targetPose) {
    // Default to no speed limit
    this(targetPose, Double.MAX_VALUE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Performing PID Align", true);
    
    // Start the alignment timer
    alignmentTimer.reset();
    alignmentTimer.start();

    var currentPose = driveSubsystem.getRobotPose();
    thetaPidController.reset(currentPose.getRotation().getRadians());
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
    double distanceToTarget = currentPose.getTranslation().getDistance(targetPose2d.getTranslation());

    double moveVelocity = MathUtil.clamp(-posPidController.calculate(distanceToTarget, 0), -maxSpeed, maxSpeed);
    double thetaVelocity = thetaPidController.calculate(currentPose.getRotation().getRadians(), targetPose2d.getRotation().getRadians());
    
    Translation2d directionVector = new Translation2d(moveVelocity, WafflesUtilities.AngleBetweenPoints(
      currentPose.getTranslation(), targetPose2d.getTranslation())
    ); 

    // SmartDashboard.putNumber("Velocity magnitude", moveVelocity);
    // SmartDashboard.putNumberArray("Direction Vector", new double[] {directionVector.getX(),directionVector.getY()});
    
    
    
    driveSubsystem.setControl(
      driveRequest
        .withDeadband(speedDeadband)
        .withRotationalDeadband(rotationDeadband)
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withVelocityX(sign * directionVector.getX())
        .withVelocityY(sign * directionVector.getY())
        .withRotationalRate(thetaVelocity)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Performing PID Align", false);

    // Stop the timer and publish the final alignment time
    alignmentTimer.stop();
    double finalAlignmentTime = alignmentTimer.get();
    alignmentTimePublisher.set(finalAlignmentTime);
    
    // Log the alignment time to SmartDashboard as well
    // SmartDashboard.putNumber("Recent Alignment Time", finalAlignmentTime);

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

  /*
   * If current pose is within a certain range of target
   */
  public static boolean isWithinAllowableRange(Pose2d target) {
    var currentPose = driveSubsystem.getRobotPose();
    return currentPose.getTranslation().getDistance(target.getTranslation()) <= PosMaxError &&
      Math.abs(currentPose.getRotation().minus(target.getRotation()).getDegrees()) < RotMaxError;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isWithinAllowableRange(targetPose2d);
  }
}
