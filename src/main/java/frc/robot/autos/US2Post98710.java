// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.data.AutoCoordinates;

public class US2Post98710 extends SequentialCommandGroup {
  public US2Post98710() {
    addCommands(
      AutoUtils.resetOdometry(AutoCoordinates.US2),
      NamedCommands.getCommand("Set Coral"),
      
      AutoUtils.prepareAndScore(AutoCoordinates.Post9, true),
      AutoUtils.intakeSequence(AutoCoordinates.Post9, AutoCoordinates.CS1),

      AutoUtils.prepareAndScore(AutoCoordinates.Post8, true),
      AutoUtils.intakeSequence(AutoCoordinates.Post8, AutoCoordinates.CS1),

      AutoUtils.prepareAndScore(AutoCoordinates.Post7, false),
      AutoUtils.intakeSequence(AutoCoordinates.Post7, AutoCoordinates.CS1),

      AutoUtils.prepareAndScore(AutoCoordinates.Post10, false),
      AutoUtils.intakeSequence(AutoCoordinates.Post10, AutoCoordinates.CS1)
    );
  }
}
