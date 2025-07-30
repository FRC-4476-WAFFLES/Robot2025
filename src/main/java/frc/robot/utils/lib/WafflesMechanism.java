// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.lib;

import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WafflesMechanism extends SubsystemBase {
  private Unit setpoint;
  private Unit constrainedSetpoint;

  private final HashMap<String, Boolean> appliedConstraints = new HashMap<String, Boolean>();

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable networkTable = inst.getTable(this.getClass().getSimpleName());
  private final DoublePublisher setpointNT = networkTable.getDoubleTopic("Setpoint").publish();
  private final DoublePublisher constrainedSetpointNT = networkTable.getDoubleTopic("ConstrainedSetpoint").publish();

  private final StringPublisher constraintsNT = networkTable.getStringTopic("Applied Constraints").publish();

  /** Creates a new WafflesMechanism. */
  public WafflesMechanism() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Apply constraints
    constrainedSetpoint = setpoint;
    applyConstraints();
    logAppliedConstraints();

    // Do stuff
  }

  /* Applies a setpoint */
  public void setSetpoint(Unit value) {
    setpoint = value;
  }

  /* Override if constraints are desired */
  public void applyConstraints() {
    // eg. runConstraint(() -> 2, "exampleConstraint");
  }

  /* Override with actual implementation */
  public boolean atSetpoint() {
    return true;
  }

  /* Publishes the currently applied constraints to networktables */
  private void logAppliedConstraints() {
    String output = "";
    for (var constraint : appliedConstraints.entrySet()) {
      if (constraint.getValue()) {
        output += constraint.getKey() + " Active\n";
      }
    }
    constraintsNT.set(output);
  }

  /* Runs a constraint */
  private void runConstraint(Supplier<Unit> constraint, String name) {
    // A constraint function returns either the setpoint, or some constrained setpoint if needed
    Unit constraintResult = constraint.get();
    appliedConstraints.put(name, constraintResult.equals(setpoint));
    constrainedSetpoint = constraintResult;
  }
}
