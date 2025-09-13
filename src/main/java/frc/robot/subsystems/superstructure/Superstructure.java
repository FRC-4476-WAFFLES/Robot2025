// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class Superstructure {
  public final Pivot pivot = new Pivot();
  public final Elevator elevator = new Elevator();

  public final Subsystem[] requirements = new Subsystem[] {pivot, elevator};

  public enum SuperstructureState {
    ZERO(45, 0),
    ALGAE_L2(130,0.65),
    ALGAE_L1(130,0.25),
    PROCESSOR(189,0.135),
    SPIT_ALGAE(140,0.2),
    CORAL_INTAKE(2.6,0),
    NET(98,1.3),
    NET_FRONT(185, 1.3),
    NET_BACK(260, 1.3),

    NET_FRONT_CLEAR(218, 1.3),
    NET_FRONT_CLEAR_FINISHED(218, 0.4),
    ALGAE_REST(218, 0.0),

    L4(160,1.10),
    L3(180,0.4),
    L2(160,0.05),

    EXECUTE_L4(140, 1.0),
    EXECUTE_L3(150, 0.3),
    EXECUTE_L2(144, 0.0),

    L1(100,0.33),

    // Maybe manual mode
    MANUAL_L4(71.0,1.1),
    MANUAL_L3(24.0,0.6772),
    MANUAL_L2(24.0,0.280),
    MANUAL_L1(150,0.33),
    
    HANDOFF_READY(35, 0.36),
    HANDOFF_EXECUTE(16, 0.17),
    HANDOFF_CLEAR(150, 0.3);

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
