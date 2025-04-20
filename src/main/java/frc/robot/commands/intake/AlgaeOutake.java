// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DynamicPathing.DynamicPathingSituation;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeOutake extends Command {
  Timer timer = new Timer();
  /** Creates a new AlgaeOutake. */
  public AlgaeOutake() {
    addRequirements(RobotContainer.intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.stop();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Controls.algaeOut.getAsBoolean()) {
      if (RobotContainer.dynamicPathingSubsystem.getCurrentPathingSituation() == DynamicPathingSituation.NET) {
        RobotContainer.intakeSubsystem.setIntakeSpeed(-280.0);
      } else {
        RobotContainer.intakeSubsystem.setIntakeSpeed(-60.0);
      }
      
      if (!RobotContainer.intakeSubsystem.isAlgaeLoaded() ) {
        timer.start();
      }
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeSubsystem.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 0.3;
  }
}
