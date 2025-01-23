// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** 
 * A convenient interface for using networktables, supports disabling networktables for the base class, or setting a custom update rate.
 * Managed through the static SubsystemNetworkManager class
 */
public interface NetworkUser {
    public void initializeNetwork();
    
    public void updateNetwork();
} 
