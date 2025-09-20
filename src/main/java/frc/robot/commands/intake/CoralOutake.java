// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.subsystems.Intake;


public class CoralOutake extends Command {
  public static final double OUTTAKE_POSITION_CHANGE = 6; // rotations

  private final Intake intakeSubsystem = RobotContainer.intakeSubsystem;
  private double outtakeEndPosition = 0;
  private double FUDGE_FACTOR = 1.8;

  private Timer simTimer = new Timer();

  /** Creates a new CoralIntake. */
  public CoralOutake() {
    addRequirements(RobotContainer.intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Make sure the intake doesn't detect us as having loaded algae in this motion
    RobotContainer.intakeSubsystem.setNoAlgaeFlag(true);
    outtakeEndPosition = intakeSubsystem.getCurrentPosition() - OUTTAKE_POSITION_CHANGE;

    // Sim
    simTimer.reset();
    simTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Spin wheels at same speed as robot back off
    var chassisSpeed = RobotContainer.driveSubsystem.getRobotChassisSpeeds();
    double wheelspeedMetersPerSecond = Math.hypot(chassisSpeed.vxMetersPerSecond, chassisSpeed.vyMetersPerSecond);
    // Slight fudge
    wheelspeedMetersPerSecond *= FUDGE_FACTOR;

    double wheelCircumference = 2 * Math.PI * PhysicalConstants.manipulatorWheelRadius.in(Meters);
    RobotContainer.intakeSubsystem.setIntakeSpeed(wheelspeedMetersPerSecond / wheelCircumference);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeSubsystem.setIntakeSpeed(0);
    RobotContainer.intakeSubsystem.setNoAlgaeFlag(false);
    
    // Get timing information from NetworkTables
    double alignmentTime = NetworkTableInstance.getDefault()
        .getTable("ScoringMetrics")
        .getEntry("FinalAlignCoral Duration")
        .getDouble(0.0);
    
    double totalScoringTime = NetworkTableInstance.getDefault()
        .getTable("ScoringMetrics")
        .getEntry("Total ScoreCoral Duration")
        .getDouble(0.0);
    
    // Record timing metrics without tracking success/failure
    RobotContainer.telemetry.recordScoringTime(alignmentTime, totalScoringTime);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (simTimer.get() > 0.2 && RobotBase.isSimulation()) {
      RobotContainer.telemetry.manipulatorCoralSimLoaded = false;
      return true;
    }

    return intakeSubsystem.getCurrentPosition() <= outtakeEndPosition || 
      (!intakeSubsystem.isCoralLoaded());
  } // && RobotBase.isSimulation()
}
