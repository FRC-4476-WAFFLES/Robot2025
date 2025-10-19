// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.fourpieceauto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignToPose;
import frc.robot.data.AutoCoordinates;

public class TemplateAuto extends SequentialCommandGroup {
  public TemplateAuto() {
    addCommands(
      new AlignToPose(AutoCoordinates.Post12)
    );
  }
}
