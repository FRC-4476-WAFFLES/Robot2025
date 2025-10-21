// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.data.AutoCoordinates;

public class OPP2Post2345 extends SequentialCommandGroup {
  public OPP2Post2345() {
    addCommands(
      NamedCommands.getCommand("Set Coral Loaded"),
      
      AutoUtils.prepareAndScore(AutoCoordinates.Post2, false),

      Commands.parallel(
        AutoUtils.driveAwayFromPost(AutoCoordinates.Post2, AutoCoordinates.CS2),
        NamedCommands.getCommand("Set Position Intake")
      ),
      NamedCommands.getCommand("Auto Coral Intake"),

      AutoUtils.prepareAndScore(AutoCoordinates.Post3, false),

      Commands.parallel(
        AutoUtils.driveAwayFromPost(AutoCoordinates.Post3, AutoCoordinates.CS2),
        NamedCommands.getCommand("Set Position Intake")
      ),
      NamedCommands.getCommand("Auto Coral Intake"),

      AutoUtils.prepareAndScore(AutoCoordinates.Post4, false),

      Commands.parallel(
        AutoUtils.driveAwayFromPost(AutoCoordinates.Post4, AutoCoordinates.CS2),
        NamedCommands.getCommand("Set Position Intake")
      ),
      NamedCommands.getCommand("Auto Coral Intake"),

      AutoUtils.prepareAndScore(AutoCoordinates.Post5, false)
    );
    AutoUtils.resetOdometry(AutoCoordinates.OPP2);
  }
}
