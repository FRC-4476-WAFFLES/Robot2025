// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.data.Constants.PhysicalConstants;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static frc.robot.RobotContainer.*;

public class DriveTeleop extends Command {
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final Supplier<Rotation2d> thetaSupplier;

  private final boolean isSetpointX, isSetpointY, isSetpointTheta;

  /* PID Controllers used if suppliers are interpreted as setpoints */
  private PIDController xPidController = new PIDController(4, 1, 0);
  private PIDController yPidController = new PIDController(4, 1, 0);
  private PIDController thetaPidController = new PIDController(9.0, 1.5, 0.0);

  /** 
   * Command that drives the robot field-oriented following velocities given by suppliers 
   */
  public DriveTeleop(DoubleSupplier xVelocitySupplier, DoubleSupplier yVelocitySupplier, Supplier<Rotation2d> thetaVelocitySupplier) {
    this(xVelocitySupplier, false, yVelocitySupplier, false, thetaVelocitySupplier, false);
  }

  /** 
   * Command that drives the robot field-oriented following velocities given by suppliers, with the option of making suppliers setpoints 
   */
  public DriveTeleop(DoubleSupplier xSupplier, boolean isSetpointX, DoubleSupplier ySupplier, boolean isSetpointY, Supplier<Rotation2d> thetaSupplier, boolean isSetpointTheta) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.thetaSupplier = thetaSupplier;

    this.isSetpointX = isSetpointX;
    this.isSetpointY = isSetpointY;
    this.isSetpointTheta = isSetpointTheta;

    thetaPidController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedDeadband = PhysicalConstants.maxSpeed * 0.05;
    double rotationDeadband = PhysicalConstants.maxAngularSpeed * 0.01;

    var currentPose = driveSubsystem.getRobotPose();
    double xVelocity = isSetpointX ? xPidController.calculate(currentPose.getX(), xSupplier.getAsDouble()) : xSupplier.getAsDouble();
    double yVelocity = isSetpointY ? yPidController.calculate(currentPose.getY(), ySupplier.getAsDouble()) : ySupplier.getAsDouble();
    double thetaVelocity = isSetpointTheta ? thetaPidController.calculate(currentPose.getRotation().getRadians(), thetaSupplier.get().getRadians()) : thetaSupplier.get().getRadians();

    driveSubsystem.setControl(
      new SwerveRequest.FieldCentric()
        .withDeadband(speedDeadband)
        .withRotationalDeadband(rotationDeadband)
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withVelocityX(xVelocity)
        .withVelocityY(yVelocity)
        .withRotationalRate(thetaVelocity)
    );

    // ChassisSpeeds targetSpeeds = new ChassisSpeeds(xVelocitySupplier.getAsDouble(), yVelocitySupplier.getAsDouble(), thetaVelocitySupplier.getAsDouble());
    
    // // Note: it is important to not discretize speeds before or after
    // // using the setpoint generator, as it will discretize them for you
    // previousSetpoint = setpointGenerator.generateSetpoint(
    //     previousSetpoint, // The previous setpoint
    //     targetSpeeds, // The desired target speeds
    //     0.02 // The loop time of the robot code, in seconds
    // );
    
    // ChassisSpeeds setpointGeneratedSpeeds = previousSetpoint.robotRelativeSpeeds();

    // // Deadband speeds
    // if (Math.sqrt(setpointGeneratedSpeeds.vxMetersPerSecond * setpointGeneratedSpeeds.vxMetersPerSecond + setpointGeneratedSpeeds.vyMetersPerSecond * setpointGeneratedSpeeds.vyMetersPerSecond) < speedDeadband) {
    //   setpointGeneratedSpeeds.vxMetersPerSecond = 0;
    //   setpointGeneratedSpeeds.vyMetersPerSecond = 0;
    // }
    // if (Math.abs(setpointGeneratedSpeeds.omegaRadiansPerSecond) < rotationDeadband) {
    //   setpointGeneratedSpeeds.omegaRadiansPerSecond = 0;
    // }

    // new SwerveRequest.ApplyFieldSpeeds()
    //   .withDriveRequestType(DriveRequestType.Velocity)
    //   .withSteerRequestType(SteerRequestType.MotionMagicExpo)
    //   .withWheelForceFeedforwardsX(previousSetpoint.feedforwards().robotRelativeForcesX())
    //   .withWheelForceFeedforwardsY(previousSetpoint.feedforwards().robotRelativeForcesY())
    //   .withSpeeds(setpointGeneratedSpeeds)
    //   .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
