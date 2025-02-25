// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.RobotContainer;
import frc.robot.data.Constants.ElevatorConstants.ElevatorLevel;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetElevatorPos extends Command {
  private final ElevatorLevel chosenLevel;

  /** Creates a new SetElevatorPos. */
  public SetElevatorPos(ElevatorLevel level) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.elevatorSubsystem);

    this.chosenLevel = level;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.elevatorSubsystem.setElevatorSetpoint(chosenLevel);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check for potential collisions
    //Elevator.CollisionType collisionPrediction = RobotContainer.elevatorSubsystem.isCollisionPredicted(chosenLevel.getHeight());
    
    // if (collisionPrediction == Elevator.CollisionType.NONE) {
    //   // Safe to move elevator
    RobotContainer.elevatorSubsystem.setElevatorSetpoint(chosenLevel);
    // } 
    // else if(collisionPrediction == Elevator.CollisionType.ENTERING_FROM_ABOVE) {
    //   // Entering above, temporarily move pivot out of the way and clamp the height to not go thru danger zone till clear
    //   RobotContainer.manipulatorSubsystem.setPivotSetpoint(PivotPosition.CLEARANCE_POSITION);
    //   RobotContainer.elevatorSubsystem.setElevatorSetpoint(ElevatorConstants.COLLISION_ZONE_UPPER);
    // }
    // else if(collisionPrediction == Elevator.CollisionType.ENTERING_FROM_BELOW) {
    //   // Entering below, temporarily move pivot out of the way and clamp the height to not go thru danger zone till clear
    //   RobotContainer.manipulatorSubsystem.setPivotSetpoint(PivotPosition.CLEARANCE_POSITION);
    //   RobotContainer.elevatorSubsystem.setElevatorSetpoint(ElevatorConstants.COLLISION_ZONE_LOWER);
    // }
    // else {
    //   // Move pivot out of the way first before moving elevator at all
    //   RobotContainer.manipulatorSubsystem.setPivotSetpoint(PivotPosition.CLEARANCE_POSITION);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // RobotContainer.elevatorSubsystem.setElevatorSetpoint(ElevatorLevel.REST_POSITION);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.elevatorSubsystem.isElevatorAtSetpoint(); // Run continuously until interrupted
  }
}
