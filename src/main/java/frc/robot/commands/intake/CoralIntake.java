// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.ManipulatorConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralIntake extends Command {
  private boolean hasDetectedCoral = false;
  private double targetPosition = 0;

  /** Creates a new CoralIntake. */
  public CoralIntake() {
    addRequirements(RobotContainer.intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.lightsSubsystem.setCoralIntakeRunning(true);
    hasDetectedCoral = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!hasDetectedCoral) {
      RobotContainer.intakeSubsystem.setIntakeSpeed(ManipulatorConstants.CORAL_INTAKE_SPEED);
      
      // Check if coral is detected for the first time
      if (RobotContainer.intakeSubsystem.isCoralLoaded()) {
        hasDetectedCoral = true;
        targetPosition = RobotContainer.intakeSubsystem.getCurrentPosition();
        RobotContainer.intakeSubsystem.setTargetPosition(targetPosition);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeSubsystem.setIntakeSpeed(0);
    // RobotContainer.intakeSubsystem.setPositionControlFlag(false);
    RobotContainer.lightsSubsystem.setCoralIntakeRunning(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.intakeSubsystem.isCoralLoaded();
  }
}
