// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.data.AutoCoordinates;

public class OPP2Lolipop extends SequentialCommandGroup {
  public OPP2Lolipop() {
    addCommands(
      AutoUtils.resetOdometry(AutoCoordinates.OPP2),
      NamedCommands.getCommand("Set Coral Loaded"),

      AutoUtils.prepareAndScore(AutoCoordinates.Post2, false),
      AutoUtils.lolipopIntakeSequence(AutoCoordinates.Post2, AutoCoordinates.LP1),

      AutoUtils.prepareAndScore(AutoCoordinates.Post3, false),
      AutoUtils.lolipopIntakeSequence(AutoCoordinates.Post3, AutoCoordinates.LP2_RIGHT),

      AutoUtils.prepareAndScore(AutoCoordinates.Post5, false),
      AutoUtils.intakeSequence(AutoCoordinates.Post5, AutoCoordinates.CS2),

      AutoUtils.prepareAndScore(AutoCoordinates.Post1, true),
      AutoUtils.intakeSequence(AutoCoordinates.Post1, AutoCoordinates.CS2)
    );
  }
}
