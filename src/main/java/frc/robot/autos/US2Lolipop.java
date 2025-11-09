// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AutoAlignToPose;
import frc.robot.data.AutoCoordinates;

public class US2Lolipop extends SequentialCommandGroup {
  public US2Lolipop() {
    addCommands(
      AutoUtils.resetOdometry(AutoCoordinates.US2),
      NamedCommands.getCommand("Set Coral Loaded"),

      AutoUtils.prepareAndScore(AutoCoordinates.Post9, true),
      AutoUtils.lolipopIntakeSequence(AutoCoordinates.Post9, AutoCoordinates.LP3),

      AutoUtils.prepareAndScore(AutoCoordinates.Post8, true),
      AutoUtils.lolipopIntakeSequence(AutoCoordinates.Post8, AutoCoordinates.LP2_LEFT),

      AutoUtils.prepareAndScore(AutoCoordinates.Post6, true),
      AutoUtils.intakeSequence(AutoCoordinates.Post6, AutoCoordinates.CS1),

      AutoUtils.prepareAndScore(AutoCoordinates.Post10, false),
      AutoUtils.intakeSequence(AutoCoordinates.Post10, AutoCoordinates.CS1)
    );
  }
}
