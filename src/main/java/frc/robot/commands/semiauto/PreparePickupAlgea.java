package frc.robot.commands.semiauto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.ScoringConstants.ScoringLevel;

/* Continuously adjusts position of elevator and pivot to desired scoring level */
public class PreparePickupAlgea extends Command {
    private final ScoringLevel level;
    /** Creates a new PrepareCoralScore. */
    public PreparePickupAlgea(ScoringLevel scoringLevel) {
        addRequirements(RobotContainer.pivotSubsystem, RobotContainer.elevatorSubsystem);
        level = scoringLevel;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.elevatorSubsystem.setElevatorSetpoint(level.getElevatorLevel());
        RobotContainer.pivotSubsystem.setPivotSetpoint(level.getPivotPosition());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return RobotContainer.pivotSubsystem.isPivotAtSetpoint() &&
                RobotContainer.elevatorSubsystem.isElevatorAtSetpoint();
    }
}
