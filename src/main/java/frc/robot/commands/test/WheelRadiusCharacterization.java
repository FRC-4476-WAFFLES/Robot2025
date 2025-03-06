// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import static frc.robot.RobotContainer.driveSubsystem;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveRotation;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;


/**
 * Test mode command that characterizes wheel radius based on gyro.
 * All rotation units are in radians.
 */
public class WheelRadiusCharacterization {
  public static final double TEST_RAMP_RATE = 0.4; // Radians / s^2
  public static final double TEST_TOP_SPEED = 0.35; // Radians / s

  public static final double TEST_DURATION = 12; // seconds
  
  // Class should not be instanciated, so constructor is private
  private WheelRadiusCharacterization() {}

  public static Command GetCharacterizationCommand() {
    // Requests for controlling drivetrain
    SysIdSwerveRotation rotationRequest = new SysIdSwerveRotation();
    ApplyRobotSpeeds stopRequest = new ApplyRobotSpeeds().withSpeeds(
      new ChassisSpeeds(0, 0, 0)
    );

    // Characterization state
    SlewRateLimiter slewRateLimiter = new SlewRateLimiter(TEST_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState(); 

    return Commands.parallel(
      // Motion sequence
      Commands.sequence(
        // Init slew rate limiter
        Commands.runOnce(() -> slewRateLimiter.reset(0)),

        // Accelerate to speed
        Commands.run(() -> {
          double rotationRate = slewRateLimiter.calculate(TEST_TOP_SPEED);
          driveSubsystem.setControl(rotationRequest.withRotationalRate(rotationRate));
        }, driveSubsystem)
      ),

      // Measurement sequence
      Commands.sequence(
        // Wait to ensure swerve modules fully align themselves
        Commands.waitSeconds(1),

        // Record initial rotation
        Commands.runOnce(() -> {
          state.startingModuleRotations = driveSubsystem.getWheelDriveRotations();
          state.accumulatedRotation = 0;
          state.lastRotation = Rotation2d.fromDegrees(driveSubsystem.getPigeon2().getYaw().getValueAsDouble());
        }),

        // Update accumuated rotation
        Commands.run(() -> {
          Rotation2d currentRotation = Rotation2d.fromDegrees(driveSubsystem.getPigeon2().getYaw().getValueAsDouble()); 
          state.accumulatedRotation += Math.abs(currentRotation.minus(state.lastRotation).getRadians());
          state.lastRotation = currentRotation;
        }).finallyDo(() -> {
          // Finally calculate radius from accumulated data
          double[] finalModuleRotations = driveSubsystem.getWheelDriveRotations();

          double accumulatedWheelRadius = 0;
          for (int i = 0; i < 4; i++) {
            // The length of the arc made by the wheel in meters
            double arcLength = driveSubsystem.getSwerveModuleRadius(i) * state.accumulatedRotation;
            double moduleDeltaRadians = Math.abs(state.startingModuleRotations[i] - finalModuleRotations[i]);

            // Arc length over theta to get radius of wheel
            accumulatedWheelRadius += arcLength / moduleDeltaRadians;
          }

          double wheelRadius = accumulatedWheelRadius / 4;
          System.out.println("==== Test Completed! ==== ");
          System.out.println("Wheel Radius Calculated As: " + wheelRadius + " m");
          System.out.println("From Gyro Yaw Delta Of: " + Units.radiansToDegrees(state.accumulatedRotation) + " deg");
          
          // NetworkTables publishers
          NetworkTable testTable = NetworkTableInstance.getDefault().getTable("WheelRadiusTest");
          DoublePublisher wheelRadiusMeterValueNT = testTable.getDoubleTopic("Wheel Radius (Meters)").publish();
          DoublePublisher wheelRadiusInchValueNT = testTable.getDoubleTopic("Wheel Radius (Inches)").publish();
          DoublePublisher gyroDeltaNT = testTable.getDoubleTopic("Gyro Test Delta (Degrees)").publish();

          wheelRadiusMeterValueNT.set(wheelRadius);
          wheelRadiusInchValueNT.set(Units.metersToInches(wheelRadius));
          gyroDeltaNT.set(Units.radiansToDegrees(state.accumulatedRotation));
        })
      )
    ).withTimeout(TEST_DURATION).finallyDo(() -> {
      // Stop drivetrain at end of characterization
      driveSubsystem.setControl(stopRequest);
    });
  }

  // Get around the fact that variables are copied into lambdas
  // Can't use primitives, so have to use a reference type class instead
  // ok
  private static class WheelRadiusCharacterizationState {
    public double[] startingModuleRotations;
    public Rotation2d lastRotation = Rotation2d.kZero;
    public double accumulatedRotation = 0.0;
  }
}
