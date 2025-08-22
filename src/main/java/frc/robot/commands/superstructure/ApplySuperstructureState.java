package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;

/* Continuously adjusts position of elevator and pivot to desired scoring level */
public class ApplySuperstructureState extends Command {
    private final SuperstructureState level;
    /** Creates a new ApplyScoringSetpoint. */
    public ApplySuperstructureState(SuperstructureState scoringLevel) {
        addRequirements(RobotContainer.superstructure.pivot, RobotContainer.superstructure.elevator);
        level = scoringLevel;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.superstructure.elevator.applySetpoint(level);
        RobotContainer.superstructure.pivot.applySetpoint(level);
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
        return RobotContainer.superstructure.pivot.atSetpoint() &&
                RobotContainer.superstructure.elevator.atSetpoint();
    }
}
