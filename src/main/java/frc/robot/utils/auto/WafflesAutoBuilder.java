// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class WafflesAutoBuilder {
    private static final ArrayList<WafflesAuto> autoList = new ArrayList<>();

    public static SendableChooser<Command> getAutoChooser() {
        var autoChooser = new SendableChooser<Command>();
        for (WafflesAuto wafflesAuto : autoList) {
            autoChooser.addOption(wafflesAuto.getClass().getSimpleName(), wafflesAuto);
        }
        return autoChooser;
    }

    public static void addAuto(WafflesAuto auto) {
        autoList.add(auto);
    }
}
