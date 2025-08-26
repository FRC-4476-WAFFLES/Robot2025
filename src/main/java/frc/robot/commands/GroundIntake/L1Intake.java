// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GroundIntake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.GroundPivotConstants.GroundPivotPosition;
import frc.robot.subsystems.GroundIntake.GroundIntakeState;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L1Intake extends Command {
  /** Creates a new GroundIntakeLeft. */
  public L1Intake() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.groundIntake);
    addRequirements(RobotContainer.groundPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.groundIntake.isCoralLeft()){
      while(!RobotContainer.groundIntake.isCoralRight()){
        RobotContainer.groundIntake.setGroundIntakeSetpoint(GroundIntakeState.SHIFT_RIGHT);
      }
    }else if(RobotContainer.groundIntake.isCoralRight()){
      while(!RobotContainer.groundIntake.isCoralLeft()){
        RobotContainer.groundIntake.setGroundIntakeSetpoint(GroundIntakeState.SHIFT_LEFT);
      }
    }else if(RobotContainer.groundIntake.isCoralMid()){
        RobotContainer.groundIntake.setGroundIntakeSetpoint(GroundIntakeState.INTAKE_MID);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    RobotContainer.groundIntake.setGroundIntakeSetpoint(GroundIntakeState.REST);
    return false;
  }
}
