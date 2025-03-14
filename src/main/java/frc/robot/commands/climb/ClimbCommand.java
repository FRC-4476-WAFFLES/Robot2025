// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.ClimberConstants.ClimberPosition;
import frc.robot.data.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.data.Constants.FunnelConstants.FunnelPosition;
import frc.robot.data.Constants.ManipulatorConstants.PivotPosition;

/* 
 * Questionable use of command based programming 
 * A command which has it's lifetime start when the state moves off of stowed, and ends once it hits stowed again
 * Handles moving all parts of the robot to conform to chosen state
 * Does not mutate state itself
*/
public class ClimbCommand extends Command {
  public enum ClimbState {
    STOWED,
    DEPLOYED,
    FIT,
    MIDDLE,
    LIFTED
  }

  /** Creates a new ClimbCommand. */
  public ClimbCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climberSubsystem, RobotContainer.funnelSubsystem, RobotContainer.pivotSubsystem, RobotContainer.elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Ensure elevator is out of the way
    RobotContainer.elevatorSubsystem.setElevatorSetpoint(ElevatorLevel.REST_POSITION);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Apply current state
    switch (RobotContainer.currentClimbState) {
      case STOWED:
        RobotContainer.climberSubsystem.setClimberPosition(ClimberPosition.ZERO);
        RobotContainer.funnelSubsystem.setFunnelPosition(FunnelPosition.DOWN);
        RobotContainer.pivotSubsystem.setPivotSetpoint(PivotPosition.ZERO);
        break;
      case DEPLOYED:
        RobotContainer.climberSubsystem.setClimberPosition(ClimberPosition.DEPLOYED);
        RobotContainer.funnelSubsystem.setFunnelPosition(FunnelPosition.UP);
        RobotContainer.pivotSubsystem.setPivotSetpoint(PivotPosition.CLIMB);
        break;
      case FIT:
        RobotContainer.climberSubsystem.setClimberPosition(ClimberPosition.FIT);
        RobotContainer.funnelSubsystem.setFunnelPosition(FunnelPosition.UP);
        RobotContainer.pivotSubsystem.setPivotSetpoint(PivotPosition.CLIMB);
        break;
      case MIDDLE:
        RobotContainer.climberSubsystem.setClimberPosition(ClimberPosition.MIDDLE);
        RobotContainer.funnelSubsystem.setFunnelPosition(FunnelPosition.UP);
        RobotContainer.pivotSubsystem.setPivotSetpoint(PivotPosition.CLIMB);
        break;
      case LIFTED:
        RobotContainer.climberSubsystem.setClimberPosition(ClimberPosition.RETRACTED);
        RobotContainer.funnelSubsystem.setFunnelPosition(FunnelPosition.UP);
        RobotContainer.pivotSubsystem.setPivotSetpoint(PivotPosition.CLIMB);
        break;
      default:
        break;
    }

    // System.out.println("AAGHHH: " + RobotContainer.currentClimbState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climberSubsystem.setClimberPosition(ClimberPosition.RETRACTED);
    RobotContainer.funnelSubsystem.setFunnelPosition(FunnelPosition.DOWN);
    RobotContainer.pivotSubsystem.setPivotSetpoint(PivotPosition.ZERO);

    // Just for safety
    RobotContainer.climberSubsystem.resetMotionProfile();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.currentClimbState == ClimbState.STOWED;
  }
}
