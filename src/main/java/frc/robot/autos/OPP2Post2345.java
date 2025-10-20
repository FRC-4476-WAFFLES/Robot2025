// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignToPose;
import frc.robot.data.AutoCoordinates;

public class OPP2Post2345 extends SequentialCommandGroup {
  public OPP2Post2345() {
    addCommands(
      NamedCommands.getCommand("Set Coral Loaded"),
      Commands.parallel(
        new AlignToPose(AutoCoordinates.Post2),
        NamedCommands.getCommand("Set Position L2")
      ),
      NamedCommands.getCommand("Autoscore L4 Right"),
      Commands.parallel(
        new AlignToPose(AutoCoordinates.CS2),
        NamedCommands.getCommand("Set Position L2")
      ),
      NamedCommands.getCommand("Auto Coral Intake"),
      Commands.parallel(
        new AlignToPose(AutoCoordinates.Post3),
        NamedCommands.getCommand("Set Position L2")
      ),
      NamedCommands.getCommand("Autoscore L4 Right")
    );
  }
}
