// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoAlignToPose;
import frc.robot.data.AutoCoordinates;
import frc.robot.utils.auto.WafflesAuto;

public class OPP2Lolipop extends WafflesAuto {
  public OPP2Lolipop() {
    addCommands(
      NamedCommands.getCommand("Set Coral Loaded"),
      Commands.parallel(
        new AutoAlignToPose(AutoCoordinates.Post2),
        NamedCommands.getCommand("Set Position L2")
      ),
      NamedCommands.getCommand("Autoscore L4 Right"),
      Commands.parallel(
        new AutoAlignToPose(AutoCoordinates.LP1),
        NamedCommands.getCommand("Set Position Intake")
      ),
      NamedCommands.getCommand("Lolipop Intake"),
      Commands.parallel(
        new AutoAlignToPose(AutoCoordinates.Post5),
        NamedCommands.getCommand("Set Position L2")
      ),
      NamedCommands.getCommand("Autoscore L4 Right"),  
      Commands.parallel(
        new AutoAlignToPose(AutoCoordinates.LP1),
        NamedCommands.getCommand("Set Position Intake")
      ),
      NamedCommands.getCommand("Auto Coral Intake"),
      Commands.parallel(
        new AutoAlignToPose(AutoCoordinates.Post6),
        NamedCommands.getCommand("Set Position L2")
      ),
      NamedCommands.getCommand("Autoscore L4 Left"),
      Commands.parallel(
        new AutoAlignToPose(AutoCoordinates.LP3),
        NamedCommands.getCommand("Set Position Intake")
      ),
      NamedCommands.getCommand("Auto Coral Intake"),
      Commands.parallel(
        new AutoAlignToPose(AutoCoordinates.Post8),
        NamedCommands.getCommand("Set Position L2")
      ),              
      NamedCommands.getCommand("Auto L4 Left")
    );
  }
}
