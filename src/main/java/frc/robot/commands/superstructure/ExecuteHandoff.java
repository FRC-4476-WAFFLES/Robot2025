package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.ManipulatorConstants;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;

/* Continuously adjusts position of elevator and pivot to desired scoring level */
public class ExecuteHandoff extends Command {
    private static final double STOP_INTAKING_TIME = 0.4;
    private enum HandoffState {
        STARTED,
        EXECUTING,
        CLEARING,
        FINISHED;
    }
    private HandoffState state = HandoffState.STARTED;
    private Timer stopIntakingTimer = new Timer();
    private Timer retryHandoffTimer = new Timer();

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
        stopIntakingTimer.stop();
        stopIntakingTimer.reset();
        retryHandoffTimer.reset();
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
                    retryHandoffTimer.restart();
                }
                break;
            
            case EXECUTING:
                RobotContainer.superstructure.applySuperstructureState(SuperstructureState.HANDOFF_EXECUTE);

                // Timeout if not at setpoint within some time, and retry pickup if coral still ready
                if (retryHandoffTimer.get() > 1.0) {
                    if (!RobotContainer.intakeSubsystem.isCoralLoaded()) {
                        state = HandoffState.STARTED;
                    } else {
                        state = HandoffState.FINISHED;
                    }
                    break;
                }

                if (RobotContainer.superstructure.atSetpoint()) {
                    state = HandoffState.CLEARING;

                    stopIntakingTimer.stop();
                    stopIntakingTimer.reset();
                }
                break;
            
            case CLEARING:
                RobotContainer.groundSuperstructure.triggerHandoff();
                RobotContainer.superstructure.applySuperstructureState(SuperstructureState.HANDOFF_CLEAR);

                // Start timer when coral is detected
                if (RobotContainer.intakeSubsystem.isCoralLoaded() && !stopIntakingTimer.isRunning()) {
                    stopIntakingTimer.restart();
                }

                // Stop intake some time after coral detection
                if (stopIntakingTimer.get() > STOP_INTAKING_TIME) {
                    RobotContainer.intakeSubsystem.setIntakeSpeed(0);
                }

                if (RobotContainer.superstructure.atSetpoint()) {
                    // If we failed to load coral and the intake is still loaded try again
                    if (!RobotContainer.intakeSubsystem.isCoralLoaded() && RobotContainer.groundSuperstructure.intake.isCoralHandoffLoaded()) {
                        state = HandoffState.STARTED;
                        break;
                    }

                    // Ensure we are at a controlled ending point for the handoff
                    state = HandoffState.FINISHED;
                }
                break;
            case FINISHED:
                break;
        }

        // System.out.println(state.toString());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        stopIntakingTimer.stop();
        stopIntakingTimer.reset();

        retryHandoffTimer.stop();
        retryHandoffTimer.reset();

        RobotContainer.intakeSubsystem.setIntakeSpeed(0);

        if (RobotBase.isSimulation()) {
            // RobotContainer.telemetry.manipulatorCoralSimLoaded = true;
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return state == HandoffState.FINISHED;
    }
}
