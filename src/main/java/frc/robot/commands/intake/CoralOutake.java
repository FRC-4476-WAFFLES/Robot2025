// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.ManipulatorConstants;
import frc.robot.data.Constants.ScoringConstants.ScoringLevel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralOutake extends Command {
  Timer timer = new Timer();
  /** Creates a new CoralIntake. */
  public CoralOutake() {
    addRequirements(RobotContainer.intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.intakeSubsystem.setNoAlgaeFlag(true);
    timer.stop();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.dynamicPathingSubsystem.getCoralScoringLevel() == ScoringLevel.L1) {
      RobotContainer.intakeSubsystem.setIntakeSpeed(18.0);
    } else {
      if (RobotContainer.dynamicPathingSubsystem.getCoralScoringLevel() == ScoringLevel.L4) {
        RobotContainer.intakeSubsystem.setIntakeSpeed(-60.0);
      } else {
        RobotContainer.intakeSubsystem.setIntakeSpeed(-28.0);
      }
    }
    
    if (!RobotContainer.intakeSubsystem.isCoralLoaded() ) {
      timer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeSubsystem.setIntakeSpeed(0);
    RobotContainer.intakeSubsystem.setNoAlgaeFlag(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 0;
  }
}
