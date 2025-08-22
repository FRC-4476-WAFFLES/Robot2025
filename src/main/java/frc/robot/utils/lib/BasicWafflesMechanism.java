// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.lib;

import java.util.HashMap;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.utils.NetworkUser;
import frc.robot.utils.SubsystemNetworkManager;

/**
 * Provides shared subsystem boilerplate.
 * Just offers a networktables implementation
 */
public class BasicWafflesMechanism extends SubsystemBase implements NetworkUser {
  private final HashMap<String, Boolean> appliedConstraints = new HashMap<String, Boolean>();

  protected final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  protected final NetworkTable networkTable = inst.getTable(this.getClass().getSimpleName());

  /** Creates a new BasicWafflesMechanism. */
  public BasicWafflesMechanism() {
    SubsystemNetworkManager.RegisterNetworkUser(this, true, CodeConstants.SUBSYSTEM_NT_UPDATE_RATE);
  }

  @Override
  public final void periodic() {
    // This shim exists to stay consistent with WafflesMechanism
    // Run actual periodic implementation
    periodicImpl();
  }

  /**
   * A direct analogue of the SubsystemBase periodic() method for use in WafflesMechanisms
   */
  protected void periodicImpl() {}

  @Override
  public void updateNetwork() {}

  @Override
  public void initializeNetwork() {}
}
