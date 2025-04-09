// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.utils.WafflesUtilities;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

public class AlignToPose extends Command {
  /* Approach Constants */
  public static final double maxAcceleration = 5.0;
  public static final double maxVelocity = 4;

  public static final double maxThetaAcceleration = 15;
  public static final double maxThetaVelocity = 4;

  // Refresh profiles if strafing more than this value
  public static final double strafeResetLimit = 0.1;
  public static final double approachFeedforwardBlendOuter = 0.05; // Distance at which velocity feedforward begins to lose influence
  public static final double approachFeedforwardBlendInner = 0.01; // Distance at which velocity feedforward loses all influence

  /* Controllers */
  private ProfiledPIDController approachPidController = new ProfiledPIDController(4.7, 0, 0.05, new Constraints(maxVelocity, maxAcceleration));
  private ProfiledPIDController thetaPidController = new ProfiledPIDController(7.0, 0, 0.1, new Constraints(maxThetaVelocity, maxThetaAcceleration));
  private SlewRateLimiter strafeRateLimiter = new SlewRateLimiter(maxAcceleration * 6);

  /* Tolerances */
  private static double PosMaxError = 0.01; // Meters
  private static Rotation2d RotMaxError = Rotation2d.fromDegrees(0.5);

  /* Data */
  private final Supplier<Pose2d> goalPoseSupplier;
  private Pose2d goalPose;

  private Trigger endTrigger;
  private double endingDebounce = 0;
  private boolean lockWheelsOnceFinished;

  private SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();
  
  /* Telemetry Variables */
  private final Timer alignmentTimer = new Timer();
  private static final NetworkTable scoringTable = NetworkTableInstance.getDefault().getTable("AlignmentMetrics");
  private static final DoublePublisher alignmentTimePublisher = scoringTable.getDoubleTopic("PID Align Duration").publish();
  private static final BooleanPublisher isAligningPublisher = scoringTable.getBooleanTopic("Performing PID Align").publish();
  private static final DoublePublisher thetaOutputPublisher = scoringTable.getDoubleTopic("Theta Output").publish();
  private static final DoublePublisher approachOutputPublisher = scoringTable.getDoubleTopic("Approach Output").publish();
  private static final DoublePublisher strafeOutputPublisher = scoringTable.getDoubleTopic("Strafe Output").publish();
  private static final DoublePublisher strafeVelocityPublisher = scoringTable.getDoubleTopic("Strafe Velocity").publish();
  private static final DoublePublisher approachVelocityPublisher = scoringTable.getDoubleTopic("Approach Velocity").publish();
  private static final DoublePublisher approachSetpointPositionPublisher = scoringTable.getDoubleTopic("Approach Setpoint Position").publish();
  private static final DoublePublisher approachSetpointVelocityPublisher = scoringTable.getDoubleTopic("Approach Setpoint Velocity").publish();
  private static final DoublePublisher distancePublisher = scoringTable.getDoubleTopic("Approach Distance").publish();
  private static final DoublePublisher feedforwardBlendPublisher = scoringTable.getDoubleTopic("Approach FF Blend").publish();
  private static final StructPublisher<Pose2d> goalPosePublisher = scoringTable.getStructTopic("Goal Pose", Pose2d.struct).publish();


  /** 
   * Command that drives the robot to align with a desired pose
   * @param targetPoseSupplier The goal pose for the robot to align to as a supplier
   */
  public AlignToPose(Supplier<Pose2d> targetPoseSupplier) {
    addRequirements(RobotContainer.driveSubsystem);

    thetaPidController.enableContinuousInput(-Math.PI, Math.PI);

    goalPoseSupplier = targetPoseSupplier;

    endTrigger = new Trigger(() -> isAtGoal())
    .debounce(endingDebounce);
  }

  /** 
   * Command that drives the robot to align with a desired pose
   * @param targetPose The goal pose for the robot to align to
   */
  public AlignToPose(Pose2d targetPose) {
    // Convenience so you don't have to write a supplier
    this(() -> targetPose);
  }

  /**
   * Sets if wheels should be locked once alignment finishes
   * @param lockWheelsOnceFinished A boolean
   */
  public void withShouldLockWheels(boolean lockWheelsOnceFinished) {
    this.lockWheelsOnceFinished = lockWheelsOnceFinished;
  }

  /**
   * Sets the time to wait once at the target before ending the command (defaults to zero) 
   * @param endingDebounce A time in seconds
   */
  public void withEndingDebounce(double endingDebounce) {
    this.endingDebounce = endingDebounce;

    endTrigger = new Trigger(() -> isAtGoal())
    .debounce(endingDebounce);
  }

  /**
   * Sets the max position tolerance for being on target (default 0.01 meters)
   * @param tolerance a value in meters
   */
  public void withPositionTolerance(double tolerance) {
    PosMaxError = tolerance;
    approachPidController.setTolerance(PosMaxError);
  }

  /**
   * Sets the max rotation tolerance for being on target (default 0.5 degrees)
   * @param tolerance a rotation
   */
  public void withThetaTolerance(Rotation2d tolerance) {
    RotMaxError = tolerance;
    thetaPidController.setTolerance(RotMaxError.getDegrees());
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isAligningPublisher.set(true);

    // Update goal pose once from supplier
    goalPose = goalPoseSupplier.get();
    
    // Start the alignment timer
    alignmentTimer.reset();
    alignmentTimer.start();

    var currentPose = RobotContainer.driveSubsystem.getRobotPose();

    // Set tolerances
    thetaPidController.setTolerance(RotMaxError.getDegrees());
    approachPidController.setTolerance(PosMaxError);

    // Reset theta controller
    thetaPidController.reset(currentPose.getRotation().getRadians(), RobotContainer.driveSubsystem.getRobotChassisSpeeds().omegaRadiansPerSecond);
    
    // Reset approach controller
    double distanceToTarget = currentPose.getTranslation().getDistance(goalPose.getTranslation());
    Translation2d velocityTowardsTarget = getVelocityTowardsTarget(currentPose, WafflesUtilities.AngleBetweenPoints(currentPose.getTranslation(), goalPose.getTranslation()));

    approachPidController.reset(distanceToTarget, 
      Math.min(
        0.0,
        -velocityTowardsTarget.getX() // Approach velocity is negative since we PID towards zero
    ));

    // Reset strafe controller
    strafeRateLimiter.reset(velocityTowardsTarget.getY());

    approachVelocityPublisher.set(velocityTowardsTarget.getX());
    strafeVelocityPublisher.set(velocityTowardsTarget.getY());
    goalPosePublisher.set(goalPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update goal pose once from supplier
    goalPose = goalPoseSupplier.get();

    Pose2d currentPose = RobotContainer.driveSubsystem.getRobotPose();
    double distanceToTarget = currentPose.getTranslation().getDistance(goalPose.getTranslation());
    Rotation2d angleToTarget = WafflesUtilities.AngleBetweenPoints(currentPose.getTranslation(), goalPose.getTranslation());
    Translation2d velocityTowardsTarget = getVelocityTowardsTarget(currentPose, angleToTarget); // This is in target space

    // Blend between feedforward and feedback control
    double approachVelocityFeedforwardBlend = MathUtil.clamp( 
      WafflesUtilities.InvLerp(approachFeedforwardBlendInner, approachFeedforwardBlendOuter, distanceToTarget),
      0, 1);
    approachVelocityFeedforwardBlend = WafflesUtilities.QuadraticEaseOut(approachVelocityFeedforwardBlend); // Square blending factor for smoothness
    feedforwardBlendPublisher.set(approachVelocityFeedforwardBlend);

    // Calculate velocity feedforward
    double approachVelocityFeedback = -approachPidController.calculate(distanceToTarget, 0);
    double approachVelocityFeedForward = -approachPidController.getSetpoint().velocity;

    // Calculate target velocities
    double targetApproachVelocity = 
      (approachVelocityFeedForward * approachVelocityFeedforwardBlend) +
      (approachVelocityFeedback * (1 - approachVelocityFeedforwardBlend));
    double targetThetaVelocity = thetaPidController.calculate(currentPose.getRotation().getRadians(), goalPose.getRotation().getRadians());
    double targetStrafeVelocity = strafeRateLimiter.calculate(0);

    // Deadband target velocities
    if (distanceToTarget < PosMaxError) {
      targetApproachVelocity = 0;
    }
    if (Math.abs(currentPose.getRotation().minus(goalPose.getRotation()).getDegrees()) < RotMaxError.getDegrees()) {
      targetThetaVelocity = 0;
    }

    // Publish telemetry
    approachSetpointPositionPublisher.set(approachPidController.getSetpoint().position);
    approachSetpointVelocityPublisher.set(approachPidController.getSetpoint().velocity);
    distancePublisher.set(distanceToTarget);

    thetaOutputPublisher.set(targetThetaVelocity);
    approachOutputPublisher.set(targetApproachVelocity);
    strafeOutputPublisher.set(targetStrafeVelocity);

    approachVelocityPublisher.set(velocityTowardsTarget.getX());
    strafeVelocityPublisher.set(velocityTowardsTarget.getY());
    goalPosePublisher.set(goalPose);

    // Convert to field velocities
    Translation2d targetFieldVelocity = new Translation2d(targetApproachVelocity, targetStrafeVelocity).rotateBy(angleToTarget); 

    // Reset motion profiling if strafing fast enough
    if (Math.abs(velocityTowardsTarget.getY()) > strafeResetLimit) {
      // Approach velocity is negative since we PID towards zero
      approachPidController.reset(distanceToTarget, 
        Math.min(
          0.0,
          -velocityTowardsTarget.getX()
      ));
      strafeRateLimiter.reset(velocityTowardsTarget.getY());
    }
    
    // Apply chosen velocity
    applyFieldVelocity(targetFieldVelocity, targetThetaVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isAligningPublisher.set(false);

    // Stop the timer and publish the final alignment time
    alignmentTimer.stop();
    double finalAlignmentTime = alignmentTimer.get();
    alignmentTimePublisher.set(finalAlignmentTime);

    if (lockWheelsOnceFinished) {
      // Stop drivetrain
      applyFieldVelocity(Translation2d.kZero, 0);
    }
  }

  /**
   * If current pose is within a certain range of target
   */
  public boolean isAtGoal() {
    var currentPose = RobotContainer.driveSubsystem.getRobotPose();

    return currentPose.getTranslation().getDistance(goalPose.getTranslation()) <= PosMaxError &&
      Math.abs(currentPose.getRotation().minus(goalPose.getRotation()).getDegrees()) < RotMaxError.getDegrees();
  }

  /**
   * Helper method to apply a chosen field velocity to the drivetrain
   */
  private void applyFieldVelocity(Translation2d targetVelocity, double targetThetaVelocity) {
    // Drivetrain deadbands
    double speedDeadband = PhysicalConstants.maxSpeed * 0.01;
    double rotationDeadband = PhysicalConstants.maxAngularSpeed * 0.003;

    // Apply swerve request
    RobotContainer.driveSubsystem.setControl(
      driveRequest
        .withDeadband(speedDeadband)
        .withRotationalDeadband(rotationDeadband)
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withVelocityX(targetVelocity.getX())
        .withVelocityY(targetVelocity.getY())
        .withRotationalRate(targetThetaVelocity)
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
    );
  }

  /**
   * Calculates current velocity towards the target pose 
   * X+ is forward
   */
  private static Translation2d getVelocityTowardsTarget(Pose2d currentPose, Rotation2d angleBetweenPoses) {
    ChassisSpeeds currentSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
      RobotContainer.driveSubsystem.getRobotChassisSpeeds(),
      currentPose.getRotation()
    );
    Translation2d fieldVelocity = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

    return fieldVelocity.rotateBy(angleBetweenPoses.unaryMinus());
  }

  @Override
  public boolean isFinished() {
    return endTrigger.getAsBoolean();
  }
}