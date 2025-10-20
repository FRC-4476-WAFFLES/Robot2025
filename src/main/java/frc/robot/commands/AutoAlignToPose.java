// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.WafflesUtilities;

/** Aligns to a pose with automatic alliance flipping & custom deadzones. For use in auto. */
public class AutoAlignToPose extends AlignToPose {
    public AutoAlignToPose(Supplier<Pose2d> targetPose) {
        super(() -> WafflesUtilities.FlipIfRedAlliance(targetPose.get()));
        applySettings();
    }

    public AutoAlignToPose(Pose2d targetPose) {
        super(WafflesUtilities.FlipIfRedAlliance(targetPose));
        applySettings();
    }

    private void applySettings() {
        withPositionTolerance(0.04);
        withThetaTolerance(Rotation2d.fromDegrees(1));
    }
}
