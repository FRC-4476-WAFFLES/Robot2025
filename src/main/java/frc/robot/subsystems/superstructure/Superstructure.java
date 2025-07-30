// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
  public enum SuperStructureState {
    ZERO(0, 0),
    CLEARANCE_POSITION(35, ),
    CLEARANCE_POSITION_ALGAE(106),
    ALGAE_L2(178.5),
    ALGAE_L1(178.5),
    PROCESSOR(189),
    SPIT_ALGAE(140),
    CORAL_INTAKE(2.6),
    NET(98),
    L4(66),
    L3(34),
    L2(34),
    L1(150),

    // Maybe manual mode
    MANUAL_L4(71.0),
    MANUAL_L3(24.0),
    MANUAL_L2(24.0),
    MANUAL_L1(150),

    NET_PREP(180);

    private final double pivotAngle;
    private final double elevatorHeight;

    SuperStructureState(double pivotAngle, double elevatorHeight) {
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


  /** Creates a new Superstructure. */
  public Superstructure() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
