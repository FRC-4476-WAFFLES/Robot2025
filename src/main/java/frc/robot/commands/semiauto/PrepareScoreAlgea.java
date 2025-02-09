package frc.robot.commands.semiauto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* Continuously adjusts position of elevator and pivot to desired scoring level */
public class PrepareScoreAlgea extends Command {
    /** Creates a new PrepareCoralScore. */
    public PrepareScoreAlgea() {
        addRequirements(RobotContainer.manipulatorSubsystem, RobotContainer.elevatorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var ScoringLevel = RobotContainer.dynamicPathingSubsystem.getCoralScoringLevel();
        RobotContainer.elevatorSubsystem.setElevatorSetpoint(ScoringLevel.getElevatorLevel());
        RobotContainer.manipulatorSubsystem.setPivotSetpoint(ScoringLevel.getPivotPosition());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return RobotContainer.manipulatorSubsystem.isPivotAtSetpoint() &&
                RobotContainer.elevatorSubsystem.isElevatorAtSetpoint() &&
                !RobotContainer.dynamicPathingSubsystem.isPathing(); // Ends only once pathing is done
    }
}
