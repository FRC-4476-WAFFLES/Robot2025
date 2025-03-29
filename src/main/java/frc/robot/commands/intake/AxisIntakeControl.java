// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls;
import frc.robot.RobotContainer;
import frc.robot.data.Constants.ManipulatorConstants;

public class AxisIntakeControl extends Command {
  private final DoubleSupplier intakeAxis;
  private final DoubleSupplier outtakeAxis;

  /** Creates a new AxisIntakeControl. */
  public AxisIntakeControl(DoubleSupplier intakeAxis, DoubleSupplier outtakeAxis) {
    addRequirements(RobotContainer.intakeSubsystem);
    this.intakeAxis = intakeAxis;
    this.outtakeAxis = outtakeAxis;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.intakeSubsystem.setIntakeSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.isOperatorOverride) {
      double intakeValue = Math.abs(intakeAxis.getAsDouble()) > Controls.AXIS_DEADBAND ? intakeAxis.getAsDouble() * ManipulatorConstants.ALGAE_INTAKE_SPEED : 0;
      double outtakeValue = Math.abs(outtakeAxis.getAsDouble()) > Controls.AXIS_DEADBAND ? -outtakeAxis.getAsDouble() * ManipulatorConstants.ALGAE_INTAKE_SPEED : 0;
      
      // Sum the values - positive for intake, negative for outtake
      RobotContainer.intakeSubsystem.setIntakeSpeed(intakeValue + outtakeValue);

      //System.out.println("Executing intake");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakeSubsystem.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
} 