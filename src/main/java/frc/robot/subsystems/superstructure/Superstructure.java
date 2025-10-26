// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utils.lib.SimpleWafflesMechanism;

public class Superstructure extends SimpleWafflesMechanism {
  public final Pivot pivot = new Pivot();
  public final Elevator elevator = new Elevator();

  public final Subsystem[] requirements = new Subsystem[] {pivot, elevator};

  public enum SuperstructureState {
    ZERO(80, 0),
    ALGAE_L2(130,0.65),
    ALGAE_L1(130,0.25),
    PROCESSOR(110,0.0),
    SPIT_ALGAE(140,0.2),
    CORAL_INTAKE(2.6,0),
    NET(98,1.3),
    NET_FRONT(185, 1.3),
    NET_BACK(260, 1.3),
    NET_PREP(218, 1.3),

    NET_FRONT_CLEAR(218, 1.3),
    NET_FRONT_CLEAR_FINISHED(218, 0.4),
    ALGAE_REST(218, 0.0),
    ALGAE_GROUND_PICKUP(83,0.0),

    GROUND_PICKUP_ALGAE(85, 0.0),
    L4(184,1),
    L3(200,0.4),
    L2(173,0.0),

    L4_FAST(184,1),

    EXECUTE_L4(145, 1.02),
    EXECUTE_L3(145, 0.4),
    EXECUTE_L2(145, 0.0),

    L1(120,0.26),

    // Maybe manual mode
    MANUAL_L4(160.0,1.35),
    MANUAL_L3(173.0,0.4),
    MANUAL_L2(173.0,0.0),
    MANUAL_L1(150,0.33),
    

    
    HANDOFF_READY(35, 0.36),
    HANDOFF_EXECUTE(18, 0.15),
    HANDOFF_CLEAR(210, 0.3);

    private final double pivotAngle;
    private final double elevatorHeight;

    SuperstructureState(double pivotAngle, double elevatorHeight) {
      this.pivotAngle = pivotAngle;
      this.elevatorHeight = elevatorHeight;
    }

    public double getPivotAngle() {
      return pivotAngle;
    }

    public double getElevatorHeight() {
      return elevatorHeight;
    }
  }

  public void applySuperstructureState(SuperstructureState state) {
    pivot.applySetpoint(state.pivotAngle);
    elevator.applySetpoint(state.elevatorHeight);
  }

  public boolean atSetpoint() {
    return pivot.atSetpoint() &&
      elevator.atSetpoint();
  }
}
