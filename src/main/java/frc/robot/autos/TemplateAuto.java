// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import frc.robot.commands.AutoAlignToPose;
import frc.robot.data.AutoCoordinates;
import frc.robot.utils.auto.WafflesAuto;

public class TemplateAuto extends WafflesAuto {
  public TemplateAuto() {
    addCommands(
      new AutoAlignToPose(AutoCoordinates.Post12)
    );
  }
}
