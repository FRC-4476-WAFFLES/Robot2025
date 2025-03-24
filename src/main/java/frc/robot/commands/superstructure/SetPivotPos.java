// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.ManipulatorConstants;
import frc.robot.data.Constants.ManipulatorConstants.PivotPosition;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetPivotPos extends Command {
  private final PivotPosition chosenPosition;

  /** Creates a new SetPivotPos. */
  public SetPivotPos(ManipulatorConstants.PivotPosition position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.pivotSubsystem);

    chosenPosition = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.pivotSubsystem.setPivotPosition(chosenPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // RobotContainer.manipulatorSubsystem.setPivotSetpoint(PivotPosition.REST_POSITION);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    return RobotContainer.pivotSubsystem.isPivotAtSetpoint();
  }
}
