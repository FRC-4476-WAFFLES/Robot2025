// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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

    /**
     * For for some reason there's no lerp in Java
     */
    public static double Lerp(double num1, double num2, double t){
        return num1 + ((num2 - num1) * t);
    }

    /**
     * Again, for some reason there's no inverse lerp in Java
     */
    public static double InvLerp(double num1, double num2, double t){
        return (t - num1)/(num2 - num1);
    }


    /**
     * Takes a pose and flips it to the other side of the field if the robot is on the red alliance.
     * @param pose The input Pose2d
     * @return The output Pose2d 
     */
    public static Pose2d FlipIfRedAlliance(Pose2d pose) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                return FlippingUtil.flipFieldPose(pose);
            }
        }
        
        return pose;
    }

    /**
     * Takes an angle and flips it to the other side of the field if the robot is on the red alliance.
     * @param angle The input angle (degrees)
     * @return The output angle 
     */
    public static Rotation2d FlipAngleIfRedAlliance(Rotation2d angle) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                return FlippingUtil.flipFieldRotation(angle);
            }
        }
        
        return angle;
    }

    /**
     * Takes an x coordinate and flips it to the other side of the field if the robot is on the red alliance.
     * @param x The x coordinate (meters)
     * @return The output coordinate 
     */
    public static double FlipXIfRedAlliance(double x) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                return FlippingUtil.fieldSizeX - x;
            }
        }
        
        return x;
    }

        /**
     * Takes an y coordinate and flips it to the other side of the field if the robot is on the red alliance.
     * @param y The y coordinate (meters)
     * @return The output coordinate 
     */
    public static double FlipYIfRedAlliance(double y) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                switch (FlippingUtil.symmetryType) {
                    case kMirrored: 
                        return y;
                    case kRotational:
                        return FlippingUtil.fieldSizeY - y;
                } 
            }
        }
        
        return y;
    }

    /**
     * Returns the angle from p1 to p2
    */
    public static double AngleBetweenPoints(Translation2d p1, Translation2d p2) {
        return Math.atan2(p2.getY() - p1.getY(), p2.getX() - p1.getX());
    }

    /**
     * Returns the angle for the driver's forward direction, depending on alliance
     * @return a rotation2d representing driver forward in field space
     */
    public static Rotation2d getDriverForwardAngle() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                return Rotation2d.k180deg;
            }
        }
        return Rotation2d.kZero;
    }
} 