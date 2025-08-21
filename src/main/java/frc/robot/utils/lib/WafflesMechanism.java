// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.lib;

import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.utils.NetworkUser;
import frc.robot.utils.SubsystemNetworkManager;

/**
 * Provides shared subsystem boilerplate.
 * Offers a constrained mechanisms system & Networktables boilerplate 
 */
public class WafflesMechanism extends SubsystemBase implements NetworkUser {
  protected double setpoint;
  protected double constrainedSetpoint;

  private final HashMap<String, Boolean> appliedConstraints = new HashMap<String, Boolean>();

  protected final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  protected final NetworkTable networkTable = inst.getTable(this.getClass().getSimpleName());
  protected final DoublePublisher setpointNT = networkTable.getDoubleTopic("Setpoint").publish();
  protected final DoublePublisher constrainedSetpointNT = networkTable.getDoubleTopic("Constrained Setpoint").publish();

  private final StringPublisher constraintsNT = networkTable.getStringTopic("Applied Constraints").publish();

  /** Creates a new WafflesMechanism. */
  public WafflesMechanism() {
    SubsystemNetworkManager.RegisterNetworkUser(this, true, CodeConstants.SUBSYSTEM_NT_UPDATE_RATE);
  }

  @Override
  public final void periodic() {
    // This method will be called once per scheduler run

    // Apply constraints
    constrainedSetpoint = setpoint;
    applyConstraints();
    logAppliedConstraints();
    constrainedSetpointNT.set(constrainedSetpoint);
    setpointNT.set(setpoint);

    // Run actual periodic implementation
    periodicImpl();
  }

  /**
   * A direct analogue of the SubsystemBase periodic() method for use in WafflesMechanisms
   */
  protected void periodicImpl() {}

  /**
   * Applies a setpoint to the mechanism
   * @param value setpoint value
   */
  public void applySetpoint(double value) {
    setpoint = value;
  }
  
  /**
   * Designed to be overridden, true by default
   * @return Is the mechanism within an allowed deadzone of it's setpoint 
   */
  public boolean atSetpoint() {
    return true;
  }
  
  /**
   * Gets the current mechanism setpoint
   */
  public double getSetpoint() {
    return setpoint;
  }

  /**
   * Should be overridden to apply constraints to the mechanism's setpoint
   * Constraints are applied with runConstraint()
   * eg. runConstraint(exampleConstraintFunc, "exampleConstraint");
   */
  protected void applyConstraints() {
    // eg. runConstraint(() -> 2, "exampleConstraint");
  }

  /**
   * Runs a constraint. Allows tracking what constraints are active for easy debugging. 
   * @param constraint the setpoint after a constraint has been applied
   * @param name the name of the constraint
   */
  protected void runConstraint(Double constraintResult, String name) {
    // A constraint function returns either the setpoint, or some constrained setpoint if needed
    appliedConstraints.put(name, constraintResult.equals(setpoint));
    constrainedSetpoint = constraintResult;
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

  @Override
  public void updateNetwork() {}

  @Override
  public void initializeNetwork() {}
}
