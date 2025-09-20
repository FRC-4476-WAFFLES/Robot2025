package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.ManipulatorConstants;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;

/* Continuously adjusts position of elevator and pivot to desired scoring level */
public class ExecuteHandoff extends Command {
    private enum HandoffState {
        STARTED,
        EXECUTING,
        CLEARING,
        FINISHED;
    }
    private HandoffState state = HandoffState.STARTED;
    private Timer timer = new Timer();

    /** Creates a new ApplyScoringSetpoint. */
    public ExecuteHandoff() {
        addRequirements(
            RobotContainer.superstructure.pivot, 
            RobotContainer.groundSuperstructure.intake, 
            RobotContainer.superstructure.elevator, 
            RobotContainer.superstructure.pivot,
            RobotContainer.intakeSubsystem
        );
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        state = HandoffState.STARTED;

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (state) {
            case STARTED:
                RobotContainer.intakeSubsystem.setIntakeSpeed(ManipulatorConstants.CORAL_INTAKE_SPEED);
                RobotContainer.superstructure.applySuperstructureState(SuperstructureState.HANDOFF_READY);
                if (RobotContainer.superstructure.atSetpoint()) {
                    // Ensure we are at a controlled starting point for the handoff
                    state = HandoffState.EXECUTING;
                }
                break;
            
            case EXECUTING:
                RobotContainer.superstructure.applySuperstructureState(SuperstructureState.HANDOFF_EXECUTE);
                if (RobotContainer.superstructure.atSetpoint()) {
                    state = HandoffState.CLEARING;
                    
                    timer.stop();
                    timer.reset();
                    timer.start();
                }
                break;
            
            case CLEARING:
                RobotContainer.groundSuperstructure.triggerHandoff();
                RobotContainer.superstructure.applySuperstructureState(SuperstructureState.HANDOFF_CLEAR);
                
                if (timer.get() > 0.3) {
                    RobotContainer.intakeSubsystem.setIntakeSpeed(0);
                }

                if (RobotContainer.superstructure.atSetpoint()) {
                    // Ensure we are at a controlled ending point for the handoff
                    state = HandoffState.FINISHED;
                }
                break;
            case FINISHED:
                RobotContainer.intakeSubsystem.setIntakeSpeed(0);
                timer.stop();
                break;
        }

        // System.out.println(state.toString());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return state == HandoffState.FINISHED;
    }
}
