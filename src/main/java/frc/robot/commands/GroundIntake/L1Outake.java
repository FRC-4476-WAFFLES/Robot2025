// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GroundIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.GroundPivotConstants.GroundPivotPosition;
import frc.robot.subsystems.GroundSuperstructure.GroundIntake.GroundIntakeState;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L1Outake extends Command {
  /** Creates a new GroundIntakeStash. */
  public L1Outake() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.groundIntake);
    addRequirements(RobotContainer.groundPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.groundPivot.setPivotPosition(GroundPivotPosition.L1);
    RobotContainer.groundIntake.setGroundIntakeSetpoint(GroundIntakeState.OUTAKE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    RobotContainer.groundIntake.setGroundIntakeSetpoint(GroundIntakeState.REST);
    RobotContainer.groundPivot.setPivotPosition(GroundPivotPosition.STOWED);
    return false;
  }
}
