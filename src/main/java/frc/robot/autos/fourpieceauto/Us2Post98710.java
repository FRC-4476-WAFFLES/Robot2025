// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.fourpieceauto;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignToPose;
import frc.robot.data.AutoCoordinates;

public class Us2Post98710 extends SequentialCommandGroup {
  public Us2Post98710() {
    addCommands(
      NamedCommands.getCommand("Set Coral"),
      Commands.parallel(
        new AlignToPose(AutoCoordinates.Post9),
        NamedCommands.getCommand("Set Position L2")
      ),
      NamedCommands.getCommand("Autoscore L4 Left"),
      Commands.parallel(
        new AlignToPose(AutoCoordinates.CS1),
        NamedCommands.getCommand("Set Position Intake")
      ),
      NamedCommands.getCommand("Auto Coral Intake"),
      Commands.parallel(
        new AlignToPose(AutoCoordinates.Post8),
        NamedCommands.getCommand("Set Position L2")
      ),
      NamedCommands.getCommand("Autoscore L4 Left"),
      Commands.parallel(
        new AlignToPose(AutoCoordinates.CS1),
        NamedCommands.getCommand("Set Position Intake")
      ),
      NamedCommands.getCommand("Auto Coral Intake"),
      Commands.parallel(
        new AlignToPose(AutoCoordinates.Post7),
        NamedCommands.getCommand("Set Position L2")
      ),
      NamedCommands.getCommand("Autoscore L4 Right"),
      Commands.parallel(
        new AlignToPose(AutoCoordinates.CS1),
        NamedCommands.getCommand("Set Position Intake")
      ),
      NamedCommands.getCommand("Auto Coral Intake"),
      Commands.parallel(
        new AlignToPose(AutoCoordinates.Post10),
        NamedCommands.getCommand("Set Position L2")
      ),
      NamedCommands.getCommand("Autoscore L4 Right")

    );
  }
}
