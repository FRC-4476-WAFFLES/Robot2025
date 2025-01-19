// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.data.Constants;
import frc.robot.subsystems.AlgaeManipulator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeIn extends Command {
  private AlgaeManipulator algaeManipulator;

  /** Creates a new AlgaeIn. */
  public AlgaeIn() {
    algaeManipulator = RobotContainer.algaeManipulator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeManipulator);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //nothing yet
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaeManipulator.setAlgaeIntakeSpeed(1);
    algaeManipulator.setOutsideTheRobot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeManipulator.setAlgaeIntakeSpeed(0);
    algaeManipulator.setInsideTheRobot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
