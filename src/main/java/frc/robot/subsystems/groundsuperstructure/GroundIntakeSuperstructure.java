// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundsuperstructure;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.GroundPivotConstants.GroundPivotPosition;
import frc.robot.subsystems.groundsuperstructure.GroundIntake.GroundIntakeState;
import frc.robot.utils.lib.SimpleWafflesMechanism;

/** Add your docs here. */
public class GroundIntakeSuperstructure extends SimpleWafflesMechanism{
    public final GroundIntake intake = new GroundIntake();
    public final GroundPivot pivot = new GroundPivot();
    public final Subsystem[] requirements = new Subsystem[] {intake, pivot};

    private boolean statemachineOverrideFlag = false;

    public enum GroundIntakeSuperstructureState {
        INTAKE_L1_STATE,
        INDEXING_L1_STATE,
        L1_READY,
        L1_SCORE_STATE,
        INTAKE_HANDOFF_STATE,
        READY_HANDOFF_STATE,
        EXECUTE_HANDOFF_STATE,
        STOWED,
        SPIT_OUT_STATE;
    }
    
    private GroundIntakeSuperstructureState currentState = GroundIntakeSuperstructureState.STOWED;
    private StringPublisher statePublisher = networkTable.getStringTopic("Current State").publish();
    private BooleanPublisher intakingHandoff = networkTable.getBooleanTopic("Intaking Handoff").publish();
    

    private Timer simTimer = new Timer();

    @Override
    protected void periodicImpl() {
        if (RobotContainer.isOperatorOverride) {
            intake.setGroundIntakeSetpoint(GroundIntakeState.OUTAKE); 
            currentState = GroundIntakeSuperstructureState.STOWED;
            return;   
        }

        if (statemachineOverrideFlag) {
            return;
        }


        switch (currentState) {
            case INTAKE_L1_STATE:
                if (intake.isCoralLeft() || intake.isCoralRight() || intake.isCoralMid()) {
                    currentState = GroundIntakeSuperstructureState.INDEXING_L1_STATE;
                } else {
                    intake.setGroundIntakeSetpoint(GroundIntakeState.INTAKE_TOP);
                    pivot.applySetpoint(GroundPivotPosition.L1_INTAKE);
                }
                break;
            
            case INDEXING_L1_STATE:
                if (intake.isCoralLeft() || intake.isCoralRight() || intake.isCoralMid()) {
                    pivot.applySetpoint(GroundPivotPosition.L1_INTAKE);
                    if(!intake.isCoralRight()){
                        intake.setGroundIntakeSetpoint(GroundIntakeState.SHIFT_RIGHT);
                    }else if(!intake.isCoralLeft()){
                        intake.setGroundIntakeSetpoint(GroundIntakeState.SHIFT_LEFT);
                    }else {
                        // We know it's centered
                        currentState = GroundIntakeSuperstructureState.L1_READY;
                    }
                } else {
                    // Lost coral, do spit out automatically
                    currentState = GroundIntakeSuperstructureState.SPIT_OUT_STATE;
                }
                break;
                
            case L1_READY:
                pivot.applySetpoint(GroundPivotPosition.L1);
                intake.setGroundIntakeSetpoint(GroundIntakeState.INTAKE_TOP_SLOW);

                if(!intake.isCoralLeft() && !intake.isCoralRight() && !intake.isCoralMid()){
                    currentState = GroundIntakeSuperstructureState.STOWED;
                }
                break;

            case L1_SCORE_STATE:
                pivot.applySetpoint(GroundPivotPosition.L1);
                if(intake.isCoralLeft() || intake.isCoralRight() || intake.isCoralMid()){
                    intake.setGroundIntakeSetpoint(GroundIntakeState.OUTAKE);
                }else{
                    currentState = GroundIntakeSuperstructureState.STOWED;
                }
                break;

            case INTAKE_HANDOFF_STATE:
                if (intake.isCoralLeft() || intake.isCoralRight() || intake.isCoralMid() || intake.isCoralHandoffLoaded()) {
                    if (intake.isCoralLeft() && intake.isCoralRight() && intake.isCoralMid()) {
                        // We grabbed front on
                        intake.setGroundIntakeSetpoint(GroundIntakeState.SHIFT_LEFT);
                    } else {
                        intake.setGroundIntakeSetpoint(GroundIntakeState.PREPARE_HANDOFF);
                    }

                    if(intake.isCoralHandoffLoaded()){
                        pivot.applySetpoint(GroundPivotPosition.HANDOFF);
                        intake.setGroundIntakeSetpoint(GroundIntakeState.REST);

                        // Only mark handoff as ready once pivot in position
                        if (pivot.atSetpoint()) {
                            currentState = GroundIntakeSuperstructureState.READY_HANDOFF_STATE;
                        }
                    } else {
                        pivot.applySetpoint(GroundPivotPosition.DEPLOYED_OFFGROUND);
                    }
                } else {
                    intake.setGroundIntakeSetpoint(GroundIntakeState.PREPARE_HANDOFF);
                    pivot.applySetpoint(GroundPivotPosition.DEPLOYED);
                }

                // Pretend intake happened in sim after 3 seconds
                if (RobotBase.isSimulation()) {
                    if (!simTimer.isRunning()) {
                        simTimer.start();
                    }
                    if (simTimer.get() > 3) {
                        // currentState = GroundIntakeSuperstructureState.READY_HANDOFF_STATE;
                        simTimer.reset();
                        simTimer.stop();
                    }
                }
            break;

            case READY_HANDOFF_STATE:
                pivot.applySetpoint(GroundPivotPosition.HANDOFF);
                intake.setGroundIntakeSetpoint(GroundIntakeState.REST);
                break; 

            case EXECUTE_HANDOFF_STATE:
                pivot.applySetpoint(GroundPivotPosition.HANDOFF);
                if(intake.isCoralHandoffLoaded() || intake.isCoralMid()){
                    if (RobotContainer.intakeSubsystem.isCoralLoaded()) {
                        intake.setGroundIntakeSetpoint(GroundIntakeState.HANDOFF);
                    } else {
                        intake.setGroundIntakeSetpoint(GroundIntakeState.REST);
                    }
                } else {
                    currentState = GroundIntakeSuperstructureState.STOWED;
                }
            break;

            case STOWED:
                pivot.applySetpoint(GroundPivotPosition.STOWED);
                intake.setGroundIntakeSetpoint(GroundIntakeState.REST);
            break;

            case SPIT_OUT_STATE:
                pivot.applySetpoint(GroundPivotPosition.DEPLOYED);
                if (intake.isCoralLeft() || intake.isCoralRight() || intake.isCoralMid() || intake.isCoralHandoffLoaded()) {
                    intake.setGroundIntakeSetpoint(GroundIntakeState.SPIT_OUT);
                } else {
                    currentState = GroundIntakeSuperstructureState.STOWED;
                }
            break;
        }

    }

    public void triggerHandoff(){
        if (isHandoffReady()){
            currentState = GroundIntakeSuperstructureState.EXECUTE_HANDOFF_STATE;
        }
    }

    public void triggerL1Score(){
        if (isL1Ready()){
            currentState = GroundIntakeSuperstructureState.L1_SCORE_STATE;
        }
    }

    public void setGroundIntakeSuperstructureSetpoint(GroundIntakeSuperstructureState setpoint) {
        currentState = setpoint;    
    }

    public boolean isHandoffReady(){
        return currentState == GroundIntakeSuperstructureState.READY_HANDOFF_STATE;
    }

    public boolean isHandoffHappening(){
        return currentState == GroundIntakeSuperstructureState.READY_HANDOFF_STATE ||
        currentState == GroundIntakeSuperstructureState.EXECUTE_HANDOFF_STATE ||
        (currentState == GroundIntakeSuperstructureState.INTAKE_HANDOFF_STATE && intake.isCoralHandoffLoaded());
    }

    public boolean isL1Ready(){
        return currentState == GroundIntakeSuperstructureState.L1_READY;
    }

    public boolean isStowed() {
        return currentState == GroundIntakeSuperstructureState.STOWED;
    }

    public GroundIntakeSuperstructureState getState() {
        return currentState;
    }

    public void setState(GroundIntakeSuperstructureState state) {
        currentState = state;
    }

    public void setStatemachineOverrideFlag(boolean val) {
        statemachineOverrideFlag = val;
    }

    /**
     * Handles the toggle, flip out & scoring logic for the L1 sequence
     */
    public void L1IntakeToggle() {
        if (currentState == GroundIntakeSuperstructureState.STOWED) {
            currentState = GroundIntakeSuperstructureState.INTAKE_L1_STATE;
        } else if (currentState == GroundIntakeSuperstructureState.INTAKE_L1_STATE) {
            currentState = GroundIntakeSuperstructureState.SPIT_OUT_STATE; // Spit out if interrupted mid intake
        } else if (currentState == GroundIntakeSuperstructureState.L1_READY || currentState == GroundIntakeSuperstructureState.INDEXING_L1_STATE) {
            triggerL1Score(); // Still try to score even if mid indexing
        }
    }

    public void handoffIntakeToggle() {
        if (currentState == GroundIntakeSuperstructureState.STOWED) {
            currentState = GroundIntakeSuperstructureState.INTAKE_HANDOFF_STATE;
        } else if (currentState == GroundIntakeSuperstructureState.INTAKE_HANDOFF_STATE) {
            currentState = GroundIntakeSuperstructureState.SPIT_OUT_STATE; // Spit out if interrupted mid intake
        }
    }

    public void startHandoffIntake() {
        if (currentState == GroundIntakeSuperstructureState.STOWED) {
            currentState = GroundIntakeSuperstructureState.INTAKE_HANDOFF_STATE;
        }
    }

    public boolean isIntakingHandoff() {
        return 
            currentState == GroundIntakeSuperstructureState.INTAKE_HANDOFF_STATE &&
            !intake.isCoralHandoffLoaded() &&
            !RobotContainer.intakeSubsystem.manipulatorLoaded();
    }
    
    
    public boolean anyCoralSensorActive() {
        return intake.isCoralHandoffLoaded() || intake.isCoralLeft() || intake.isCoralMid() || intake.isCoralRight();
    }

    public boolean isIntaking() {
        return currentState == GroundIntakeSuperstructureState.INTAKE_HANDOFF_STATE || currentState == GroundIntakeSuperstructureState.INTAKE_L1_STATE;
    }

    @Override
    public void updateNetwork() {
        statePublisher.set(currentState.toString());
        intakingHandoff.set(isIntakingHandoff());
    }
}
