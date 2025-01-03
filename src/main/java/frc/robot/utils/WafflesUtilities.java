// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.hardware.TalonFX;

public class WafflesUtilities {
    /**
     * Private constructor to prevent instantiation of utility class
     */
    private WafflesUtilities() {
        throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
    }

    /**
     * Checks if a TalonFX motor's stator current is over a specified threshold
     * @param talonFX The TalonFX motor to check
     * @param currentThreshold The current threshold to compare against
     * @return true if the stator current is over the threshold, false otherwise
     */
    public static boolean isOverThreshold(TalonFX talonFX, double currentThreshold) {
        return talonFX.getStatorCurrent().getValueAsDouble() > currentThreshold;
    }
} 