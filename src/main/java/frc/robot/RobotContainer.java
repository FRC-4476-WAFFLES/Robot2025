// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.HashSet;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.ManualElevatorControl;
import frc.robot.commands.ResetGyroHeading;
import frc.robot.commands.SetElevatorPos;
import frc.robot.data.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.data.Constants.ManipulatorConstants.PivotPosition;
import frc.robot.data.Constants.ScoringConstants.ScoringLevel;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.data.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DynamicPathingSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Telemetry;
import frc.robot.commands.AxisIntakeControl;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.SetPivotPos;
import frc.robot.commands.DefaultPosition;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  /* Subsystems */
  public static final DriveSubsystem driveSubsystem = TunerConstants.createDrivetrain();
  public static final Manipulator manipulatorSubsystem = new Manipulator();
  public static final Elevator elevatorSubsystem = new Elevator();
  // public static final Funnel funnelSubsystem = new Funnel();
  public static final Climber climberSubsystem = null; //  new Climber()

  public static final DynamicPathingSubsystem dynamicPathingSubsystem = new DynamicPathingSubsystem();
  private static final Telemetry telemetry = new Telemetry(PhysicalConstants.maxSpeed);

  /* Commands */
  private final ManualElevatorControl manualElevatorControl = new ManualElevatorControl();
  private final AxisIntakeControl axisIntakeControl = new AxisIntakeControl(
    Controls.operatorController::getRightTriggerAxis,
    Controls.operatorController::getLeftTriggerAxis
  );
  private final ResetGyroHeading resetGyroHeading = new ResetGyroHeading();
  private final DefaultPosition defaultPosition = new DefaultPosition();

  /* Global Robot State */
  private final SendableChooser<Command> autoChooser;
  public static boolean isOperatorOverride = false;

  /** The static entry point for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Swerve telemetry from odometry thread
    driveSubsystem.registerTelemetry(telemetry::telemeterize);
    driveSubsystem.setDefaultCommand(new DriveTeleop(
      Controls::getDriveY,
      Controls::getDriveX,
      Controls::getDriveRotation
    ));

    // Set default commands based on mode
    new Trigger(() -> !isOperatorOverride)
        .whileTrue(manualElevatorControl);
    
    new Trigger(() -> isOperatorOverride)
        .whileTrue(defaultPosition);

    // Register commands to be used by pathplanner autos
    registerNamedCommands();

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Warmup pathplanner to reduce delay when dynamic pathing
    FollowPathCommand.warmupCommand().schedule();
  }

  private void configureBindings() {
    // Create conditional triggers based on operator override state
    Trigger inNormalMode = new Trigger(() -> !isOperatorOverride);
    Trigger inOverrideMode = new Trigger(() -> isOperatorOverride);

    // Toggle operator override
    Controls.operatorController.start().onTrue(
      new InstantCommand(RobotContainer::toggleOperatorOverride)
    );

    Controls.rightJoystick.button(9).whileTrue(resetGyroHeading);

    // Normal mode button bindings
    inNormalMode.and(Controls.operatorController.b()).onTrue(new InstantCommand(() -> { dynamicPathingSubsystem.setCoralScoringLevel(ScoringLevel.L1); }));
    inNormalMode.and(Controls.operatorController.a()).onTrue(new InstantCommand(() -> { dynamicPathingSubsystem.setCoralScoringLevel(ScoringLevel.L2); }));
    inNormalMode.and(Controls.operatorController.x()).onTrue(new InstantCommand(() -> { dynamicPathingSubsystem.setCoralScoringLevel(ScoringLevel.L3); }));
    inNormalMode.and(Controls.operatorController.y()).onTrue(new InstantCommand(() -> { dynamicPathingSubsystem.setCoralScoringLevel(ScoringLevel.L4); }));
    
    // Override mode immediately moves to position while held
    inOverrideMode.and(Controls.operatorController.b()).whileTrue(
        Commands.parallel(
            new SetElevatorPos(ElevatorLevel.L1),
            new SetPivotPos(PivotPosition.L1)
        ));
    inOverrideMode.and(Controls.operatorController.x()).whileTrue(
        Commands.parallel(
            new SetElevatorPos(ElevatorLevel.L2),
            new SetPivotPos(PivotPosition.L2)
        ));

    inOverrideMode.and(Controls.operatorController.y()).whileTrue(
        Commands.parallel(
            new SetElevatorPos(ElevatorLevel.L3),
            new SetPivotPos(PivotPosition.L3)
        ));

    inOverrideMode.and(Controls.operatorController.a()).whileTrue(
        Commands.parallel(
            new SetElevatorPos(ElevatorLevel.L4),
            new SetPivotPos(PivotPosition.L4)
        ));

    // Axis intake control only in override mode
    inOverrideMode.whileTrue(axisIntakeControl);

    inNormalMode.and(Controls.operatorController.rightStick()).whileTrue(new CoralIntake());

    // Dynamic path to coral scoring
    Controls.rightJoystick.button(1).whileTrue(Commands.defer(
      () -> dynamicPathingSubsystem.getCurrentDynamicPathCommand(), new HashSet<>(Arrays.asList(driveSubsystem))
    ));

    Controls.operatorController.povRight().onTrue(
      new InstantCommand(
        () -> {dynamicPathingSubsystem.setCoralScoringSide(true);}
      )
    );
    Controls.operatorController.povLeft().onTrue(
      new InstantCommand(
        () -> {dynamicPathingSubsystem.setCoralScoringSide(false);}
      )
    );
  }

  private static void toggleOperatorOverride() {
    isOperatorOverride = !isOperatorOverride;
    telemetry.publishOperatorOverrideInfo();
  }

  /**
   * Use this method to define name->command mappings. Names will be used by PathPlanner to 
   * call commands in full autos. 
   */
  private void registerNamedCommands() {
    // Register Named Commands
    // Add other commands to be able to run them in autos
    // NamedCommands.registerCommand("exampleCommand", exampleCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
