// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
  public enum SuperStructureState {
    ZERO(0, 0),
    ALGAE_L2(178.5,0.88),
    ALGAE_L1(178.5,0.54),
    PROCESSOR(189,0.135),
    SPIT_ALGAE(140,0.2),//elevator height is random here
    CORAL_INTAKE(2.6,0),
    NET(98,1.5),
    L4(66,1.50),
    L3(34,0.865),
    L2(34,0.44),
    L1(150,0.33),

    // Maybe manual mode
    MANUAL_L4(71.0,1.440),
    MANUAL_L3(24.0,0.6772),
    MANUAL_L2(24.0,0.280),
    MANUAL_L1(150,0.33),

    NET_PREP(180,1.3);

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
