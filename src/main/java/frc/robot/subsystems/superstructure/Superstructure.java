// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;

public class Superstructure {
  public final Pivot pivot = new Pivot();
  public final Elevator elevator = new Elevator();

  public final Subsystem[] requirements = new Subsystem[] {pivot, elevator};

  public enum SuperstructureState {
    ZERO(0, 0),
    ALGAE_L2(178.5,0.88),
    ALGAE_L1(178.5,0.54),
    PROCESSOR(189,0.135),
    SPIT_ALGAE(140,0.2),
    CORAL_INTAKE(2.6,0),
    NET(98,1.5),
    NET_FRONT(98, 1.5),
    NET_BACK(98, 1.5),
    L4(100,1.50),
    L3(100,0.865),
    L2(100,0.44),
    L1(100,0.33),

    // Maybe manual mode
    MANUAL_L4(71.0,1.440),
    MANUAL_L3(24.0,0.6772),
    MANUAL_L2(24.0,0.280),
    MANUAL_L1(150,0.33),
    
    HANDOFF_READY(15, 0.4),
    HANDOFF_EXECUTE(15, 0.2),
    HANDOFF_CLEAR(30, 0.3);

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
