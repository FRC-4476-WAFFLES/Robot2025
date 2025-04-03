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
import frc.robot.RobotContainer;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.utils.WafflesUtilities;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AlignToPose extends Command {
  /* Approach Constants */
  public static final double maxAcceleration = 5.0;
  public static final double maxVelocity = 4;

  public static final double maxThetaAcceleration = 15;
  public static final double maxThetaVelocity = 4;

  // Refresh profiles if strafing more than this value
  public static final double strafeResetLimit = 0.1;
  public static final double approachFeedforwardBlendOuter = 0.2; // Distance at which velocity feedforward begins to lose influence
  public static final double approachFeedforwardBlendInner = 0.1; // Distance at which velocity feedforward loses all influence

  /* Controllers */
  private ProfiledPIDController approachPidController = new ProfiledPIDController(4.7, 0, 0.2, new Constraints(maxVelocity, maxAcceleration));
  private ProfiledPIDController thetaPidController = new ProfiledPIDController(7.0, 0, 0.1, new Constraints(maxThetaVelocity, maxThetaAcceleration));
  private SlewRateLimiter strafeRateLimiter = new SlewRateLimiter(maxAcceleration);

  /* Constants */
  private static final double PosMaxError = 0.01;
  private static final double RotMaxError = 0.5; // degrees

  /* Data */
  private SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();
  private final Pose2d targetPose;
  private final double maxSpeed;
  private final boolean stopOnceDone;

  private Trigger endTrigger;
  
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


  /** 
   * Command that drives the robot to align with coral scoring
   */
  public AlignToPose(Pose2d targetPose, double maxSpeed, double endingDebounce, boolean lockOnceDone) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveSubsystem);

    thetaPidController.enableContinuousInput(-Math.PI, Math.PI);

    this.targetPose = targetPose;
    this.maxSpeed = maxSpeed;
    this.stopOnceDone = lockOnceDone;

    endTrigger = new Trigger(() -> isWithinRangeOfTarget(targetPose))
    .debounce(endingDebounce);
  }

  /** 
   * Command that drives the robot to align with coral scoring
   */
  public AlignToPose(Pose2d targetPose, double maxSpeed, double endingDebounce) {
    // Default to no speed limit
    this(targetPose, maxSpeed, endingDebounce, true);
  }

  /** 
   * Command that drives the robot to align with coral scoring
   */
  public AlignToPose(Pose2d targetPose, double maxSpeed) {
    // Default to no speed limit
    this(targetPose, maxSpeed, 0, true);
  }

  /** 
   * Command that drives the robot to align with coral scoring
   */
  public AlignToPose(Pose2d targetPose) {
    // Default to no speed limit
    this(targetPose, Double.MAX_VALUE, 0, true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isAligningPublisher.set(true);
    
    // Start the alignment timer
    alignmentTimer.reset();
    alignmentTimer.start();

    var currentPose = RobotContainer.driveSubsystem.getRobotPose();

    // Set tolerances
    thetaPidController.setTolerance(RotMaxError);
    approachPidController.setTolerance(PosMaxError);

    // Reset theta controller
    thetaPidController.reset(currentPose.getRotation().getRadians(), RobotContainer.driveSubsystem.getRobotChassisSpeeds().omegaRadiansPerSecond);
    
    // Reset approach controller
    double distanceToTarget = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    Translation2d velocityTowardsTarget = getVelocityTowardsTarget(currentPose, WafflesUtilities.AngleBetweenPoints(currentPose.getTranslation(), targetPose.getTranslation()));

    approachPidController.reset(distanceToTarget, 
      Math.min(
        0.0,
        -velocityTowardsTarget.getX() // Approach velocity is negative since we PID towards zero
    ));

    // Reset strafe controller
    strafeRateLimiter.reset(velocityTowardsTarget.getY());

    approachVelocityPublisher.set(velocityTowardsTarget.getX());
    strafeVelocityPublisher.set(velocityTowardsTarget.getY());


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var currentPose = RobotContainer.driveSubsystem.getRobotPose();
    double distanceToTarget = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    Rotation2d angleToTarget = WafflesUtilities.AngleBetweenPoints(currentPose.getTranslation(), targetPose.getTranslation());

    // Calculate velocity feedforward
    double approachVelocityFeedback = MathUtil.clamp(-approachPidController.calculate(distanceToTarget, 0), -maxSpeed, maxSpeed);
    double approachVelocityFeedforwardBlend = MathUtil.clamp( 
      WafflesUtilities.InvLerp(distanceToTarget, approachFeedforwardBlendInner, approachFeedforwardBlendOuter),
      0, 1);
    if (distanceToTarget <= approachFeedforwardBlendInner) {
      approachVelocityFeedforwardBlend = 0;
    }

    double approachVelocityFeedForward = -approachPidController.getSetpoint().velocity * approachVelocityFeedforwardBlend;
    
    feedforwardBlendPublisher.set(approachVelocityFeedforwardBlend);

    approachVelocityFeedback *= 1 - approachVelocityFeedforwardBlend;

    // Calculate target velocities
    double targetApproachVelocity = approachVelocityFeedForward + approachVelocityFeedback;
    double targetThetaVelocity = thetaPidController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    double targetStrafeVelocity = strafeRateLimiter.calculate(0);

    // Deadband target velocities
    if (distanceToTarget < PosMaxError) {
      targetApproachVelocity = 0;
    }
    if (Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees()) < RotMaxError) {
      targetThetaVelocity = 0;
    }

    approachSetpointPositionPublisher.set(approachPidController.getSetpoint().position);
    approachSetpointVelocityPublisher.set(approachPidController.getSetpoint().velocity);
    distancePublisher.set(distanceToTarget);

    // Publish telemetry
    thetaOutputPublisher.set(targetThetaVelocity);
    approachOutputPublisher.set(targetApproachVelocity);
    strafeOutputPublisher.set(targetStrafeVelocity);

    // Convert to field velocities
    Translation2d targetFieldVelocity = new Translation2d(targetApproachVelocity, targetStrafeVelocity).rotateBy(angleToTarget); 

    // Reset motion profiling if strafing fast enough
    Translation2d velocityTowardsTarget = getVelocityTowardsTarget(currentPose, angleToTarget);
    if (velocityTowardsTarget.getY() > strafeResetLimit) {
      // Approach velocity is negative since we PID towards zero
      approachPidController.reset(distanceToTarget, 
        Math.min(
          0.0,
          -velocityTowardsTarget.getX()
      ));
      strafeRateLimiter.reset(velocityTowardsTarget.getY());
    }

    approachVelocityPublisher.set(velocityTowardsTarget.getX());
    strafeVelocityPublisher.set(velocityTowardsTarget.getY());
    
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

    if (stopOnceDone) {
      // Stop drivetrain
      applyFieldVelocity(Translation2d.kZero, 0);
    }
  }

  /*
   * If current pose is within a certain range of target
   */
  public static boolean isWithinRangeOfTarget(Pose2d target) {
    var currentPose = RobotContainer.driveSubsystem.getRobotPose();
    return currentPose.getTranslation().getDistance(target.getTranslation()) <= PosMaxError &&
      Math.abs(currentPose.getRotation().minus(target.getRotation()).getDegrees()) < RotMaxError;
  }

  /*
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

  /*
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

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endTrigger.getAsBoolean();
  }
}