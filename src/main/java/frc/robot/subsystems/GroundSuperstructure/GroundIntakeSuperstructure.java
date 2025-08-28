// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GroundSuperstructure;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.GroundPivotConstants.GroundPivotPosition;
import frc.robot.subsystems.GroundSuperstructure.GroundIntake.GroundIntakeState;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.Pivot;
import frc.robot.utils.lib.SimpleWafflesMechanism;

/** Add your docs here. */
public class GroundIntakeSuperstructure extends SimpleWafflesMechanism{
    public final GroundIntake groundIntake = new GroundIntake();
    public final GroundPivot groundPivot = new GroundPivot();
    public final Subsystem[] requirements = new Subsystem[] {groundIntake, groundPivot};
    public enum GroundIntakeSuperstructureState {
        INTAKE_L1_STATE,
        L1_READY,
        L1_SCORE_STATE,
        INTAKE_HANDOFF_STATE,
        READY_HANDOFF_STATE,
        EXECUTE_HANDOFF_STATE,
        STOWED,
        SPIT_OUT_STATE;
    }
    private GroundIntakeSuperstructureState currentState;
    @Override
    protected void periodicImpl() {
        switch (currentState) {
            case INTAKE_L1_STATE:
                if (groundIntake.isCoralLeft() || groundIntake.isCoralRight() || groundIntake.isCoralMid()) {
                    groundPivot.applySetpoint(GroundPivotPosition.L1);
                    if(!groundIntake.isCoralRight()){
                        groundIntake.setGroundIntakeSetpoint(GroundIntakeState.SHIFT_RIGHT);
                    }else if(!groundIntake.isCoralLeft()){
                        groundIntake.setGroundIntakeSetpoint(GroundIntakeState.SHIFT_LEFT);
                    }else {
                        // We know it's centered
                        currentState = GroundIntakeSuperstructureState.L1_READY;
                    }
                } else {
                    groundIntake.setGroundIntakeSetpoint(GroundIntakeState.INTAKE_TOP);
                    groundPivot.applySetpoint(GroundPivotPosition.DEPLOYED);
                }
                break;
                
            case L1_READY:
                groundPivot.applySetpoint(GroundPivotPosition.L1);
                groundIntake.setGroundIntakeSetpoint(GroundIntakeState.INTAKE_TOP);
                currentState = GroundIntakeSuperstructureState.L1_SCORE_STATE;
                break;

            case L1_SCORE_STATE:
                groundPivot.applySetpoint(GroundPivotPosition.L1);
                if(groundIntake.isCoralLeft() || groundIntake.isCoralRight() || groundIntake.isCoralMid()){
                    groundIntake.setGroundIntakeSetpoint(GroundIntakeState.OUTAKE);
                }else{
                    currentState = GroundIntakeSuperstructureState.STOWED;
                }
                break;

            case INTAKE_HANDOFF_STATE:
                if (groundIntake.isCoralLeft() || groundIntake.isCoralRight() || groundIntake.isCoralMid()) {
                    groundPivot.applySetpoint(GroundPivotPosition.HANDOFF);
                    if(groundIntake.isCoralLoaded()){
                        currentState = GroundIntakeSuperstructureState.READY_HANDOFF_STATE;
                    }else{
                        groundIntake.setGroundIntakeSetpoint(GroundIntakeState.PREPARE_HANDOFF);
                    }
                } else {
                    groundIntake.setGroundIntakeSetpoint(GroundIntakeState.INTAKE_TOP);
                    groundPivot.applySetpoint(GroundPivotPosition.DEPLOYED);
                }
            break;

            case READY_HANDOFF_STATE:
                groundPivot.applySetpoint(GroundPivotPosition.HANDOFF);
                groundIntake.setGroundIntakeSetpoint(GroundIntakeState.INTAKE_TOP);
                break; 

            case EXECUTE_HANDOFF_STATE:
                groundPivot.applySetpoint(GroundPivotPosition.HANDOFF);
                if(groundIntake.isCoralLoaded()){
                groundIntake.setGroundIntakeSetpoint(GroundIntakeState.HANDOFF);
                }
            break;

            case STOWED:
                groundPivot.applySetpoint(GroundPivotPosition.STOWED);
                groundIntake.setGroundIntakeSetpoint(GroundIntakeState.REST);
            break;

            case SPIT_OUT_STATE:
                groundPivot.applySetpoint(GroundPivotPosition.DEPLOYED);
                if (groundIntake.isCoralLeft() || groundIntake.isCoralRight() || groundIntake.isCoralMid()) {
                    groundIntake.setGroundIntakeSetpoint(GroundIntakeState.OUTAKE);
                }
        }

    }

    public void triggerHandoff(){
        if (isHandoffReady()){
            currentState = GroundIntakeSuperstructureState.READY_HANDOFF_STATE;
        }
    }
    public void setGroundIntakeSuperstructureSetpoint(GroundIntakeSuperstructureState setpoint) {
        currentState = setpoint;    
    }

    public boolean isHandoffReady(){
        return currentState == GroundIntakeSuperstructureState.READY_HANDOFF_STATE;
    }
}
