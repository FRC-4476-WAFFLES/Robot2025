// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.data.AutoCoordinates;
import frc.robot.commands.drive.AutoAlignToPose;

public class US2Post98710 extends SequentialCommandGroup {
  public US2Post98710() {
    addCommands(
      NamedCommands.getCommand("Set Coral"),
      
      AutoUtils.prepareAndScore(AutoCoordinates.Post9, false),

      Commands.parallel(
        AutoUtils.driveAwayFromPost(AutoCoordinates.Post9, AutoCoordinates.CS1),
        NamedCommands.getCommand("Set Position Intake")
      ),
      NamedCommands.getCommand("Auto Coral Intake"),

      AutoUtils.prepareAndScore(AutoCoordinates.Post8, false),

      Commands.parallel(
        AutoUtils.driveAwayFromPost(AutoCoordinates.Post8, AutoCoordinates.CS1),
        NamedCommands.getCommand("Set Position Intake")
      ),
      NamedCommands.getCommand("Auto Coral Intake"),

      AutoUtils.prepareAndScore(AutoCoordinates.Post7, false),

      Commands.parallel(
        AutoUtils.driveAwayFromPost(AutoCoordinates.Post7, AutoCoordinates.CS1),
        NamedCommands.getCommand("Set Position Intake")
      ),
      NamedCommands.getCommand("Auto Coral Intake"),

      AutoUtils.prepareAndScore(AutoCoordinates.Post10, false)
    );
  }
}
