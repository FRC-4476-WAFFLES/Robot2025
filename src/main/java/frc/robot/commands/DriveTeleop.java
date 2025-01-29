// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.data.Constants.PhysicalConstants;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import static frc.robot.RobotContainer.*;

public class DriveTeleop extends Command {
  private final DoubleSupplier xVelocitySupplier;
  private final DoubleSupplier yVelocitySupplier;
  private final DoubleSupplier thetaVelocitySupplier;

  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;

  private SwerveRequest.ApplyRobotSpeeds driveRequest = new SwerveRequest.ApplyRobotSpeeds();

  /** 
   * Command that drives the robot field-oriented following velocities given by suppliers 
   */
  public DriveTeleop(DoubleSupplier xVelocitySupplier, DoubleSupplier yVelocitySupplier, DoubleSupplier thetaVelocitySupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  
    this.xVelocitySupplier = xVelocitySupplier;
    this.yVelocitySupplier = yVelocitySupplier;
    this.thetaVelocitySupplier = thetaVelocitySupplier;

    setpointGenerator = new SwerveSetpointGenerator(
      driveSubsystem.PathPlannerConfig, // The robot configuration. This is the same config used for generating trajectories and running path following commands.
      PhysicalConstants.maxAngularSpeed // The max rotation velocity of a swerve module in radians per second. This should probably be stored in your Constants file
    );

    // Initialize the previous setpoint to the robot's current speeds & module states
    ChassisSpeeds currentSpeeds = driveSubsystem.getCurrentRobotChassisSpeeds(); // Method to get current robot-relative chassis speeds
    
    var modules = driveSubsystem.getModules();
    SwerveModuleState[] currentStates = new SwerveModuleState[4];
    for (int i = 0; i < modules.length; i++) {
      currentStates[i] = modules[i].getCurrentState();
    }

    previousSetpoint = new SwerveSetpoint(currentSpeeds, currentStates, DriveFeedforwards.zeros(driveSubsystem.PathPlannerConfig.numModules));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedDeadband = PhysicalConstants.maxSpeed * 0.05;
    double rotationDeadband = PhysicalConstants.maxAngularSpeed * 0.01;

    // driveSubsystem.setControl(
    //   new SwerveRequest.FieldCentric()
    //     .withDeadband(speedDeadband)
    //     .withRotationalDeadband(rotationDeadband)
    //     .withDriveRequestType(DriveRequestType.Velocity)
    //     .withSteerRequestType(SteerRequestType.MotionMagicExpo)
    //     .withVelocityX(xVelocitySupplier.getAsDouble())
    //     .withVelocityY(yVelocitySupplier.getAsDouble())
    //     .withRotationalRate(thetaVelocitySupplier.getAsDouble())
    // );

    ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(xVelocitySupplier.getAsDouble(), yVelocitySupplier.getAsDouble(), thetaVelocitySupplier.getAsDouble());

    Translation2d speeds = new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
    speeds.rotateBy(driveSubsystem.getOperatorForwardDirection());
    
    fieldRelativeSpeeds.vxMetersPerSecond = speeds.getX();
    fieldRelativeSpeeds.vyMetersPerSecond = speeds.getY();

    ChassisSpeeds targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      fieldRelativeSpeeds,
      driveSubsystem.getRobotPose().getRotation()
    );



    // Note: it is important to not discretize speeds before or after
    // using the setpoint generator, as it will discretize them for you
    previousSetpoint = setpointGenerator.generateSetpoint(
      previousSetpoint, // The previous setpoint
      targetSpeeds, // The desired target speeds
      0.02 // The loop time of the robot code, in seconds
    );

    
    
    ChassisSpeeds setpointGeneratedSpeeds = previousSetpoint.robotRelativeSpeeds();

    // Deadband speeds
    if (Math.sqrt(setpointGeneratedSpeeds.vxMetersPerSecond * setpointGeneratedSpeeds.vxMetersPerSecond + setpointGeneratedSpeeds.vyMetersPerSecond * setpointGeneratedSpeeds.vyMetersPerSecond) < speedDeadband) {
      setpointGeneratedSpeeds.vxMetersPerSecond = 0;
      setpointGeneratedSpeeds.vyMetersPerSecond = 0;
    }
    if (Math.abs(setpointGeneratedSpeeds.omegaRadiansPerSecond) < rotationDeadband) {
      setpointGeneratedSpeeds.omegaRadiansPerSecond = 0;
    }

    driveSubsystem.setControl(
      driveRequest
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withWheelForceFeedforwardsX(previousSetpoint.feedforwards().robotRelativeForcesX())
        .withWheelForceFeedforwardsY(previousSetpoint.feedforwards().robotRelativeForcesY())
        .withSpeeds(setpointGeneratedSpeeds)
    );

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
