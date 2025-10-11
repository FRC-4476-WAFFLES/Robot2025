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
        timer.stop();
        timer.reset();
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
                    timer.restart();
                }
                break;
            
            case EXECUTING:
                RobotContainer.superstructure.applySuperstructureState(SuperstructureState.HANDOFF_EXECUTE);

                // Timeout if not at setpoint within 1.5 seconds, and retry pickup if coral still ready
                if (timer.get() > 1.5) {
                    if (RobotContainer.groundSuperstructure.isHandoffReady() &&
                        !RobotContainer.intakeSubsystem.isAlgaeLoaded() &&
                        !RobotContainer.intakeSubsystem.isCoralLoaded()) {
                        state = HandoffState.STARTED;
                    } else {
                        state = HandoffState.FINISHED;
                    }
                    break;
                }

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

                // Start timer when coral is detected
                if (RobotContainer.intakeSubsystem.isCoralLoaded() && !timer.hasElapsed(0)) {
                    timer.restart();
                }

                // Stop intake 0.15s after coral detection
                if (timer.get() > 0.4) {
                    RobotContainer.intakeSubsystem.setIntakeSpeed(0);
                }

                if (RobotContainer.superstructure.atSetpoint() && timer.get() > 0.15) {
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
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return state == HandoffState.FINISHED;
    }
}
