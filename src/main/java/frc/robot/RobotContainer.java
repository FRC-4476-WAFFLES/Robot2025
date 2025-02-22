// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.HashSet;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.GenericHID;

import frc.robot.commands.DriveTeleop;
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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Telemetry;
import frc.robot.commands.AlgeaOutake;
import frc.robot.commands.AxisIntakeControl;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.SetPivotPos;
import frc.robot.commands.semiauto.ApplyScoringSetpoint;
import frc.robot.commands.semiauto.ScoreCoral;
import frc.robot.commands.DefaultPosition;
import frc.robot.subsystems.Lights;
import frc.robot.commands.DefaultLightCommand;
import frc.robot.commands.ZeroMechanisms;
import frc.robot.subsystems.MechanismPoses;


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
  public static final Pivot pivotSubsystem = new Pivot();
  public static final Intake intakeSubsystem = new Intake();
  public static final Elevator elevatorSubsystem = new Elevator();
  // public static final Funnel funnelSubsystem = new Funnel();
  public static final Climber climberSubsystem = null; //  new Climber()
  public static final Lights lightsSubsystem = new Lights();
  public static final MechanismPoses mechanismPoses = new MechanismPoses();

  public static final DynamicPathingSubsystem dynamicPathingSubsystem = new DynamicPathingSubsystem();
  public static final Telemetry telemetry = new Telemetry(PhysicalConstants.maxSpeed);

  /* Commands */
  private final AxisIntakeControl axisIntakeControl = new AxisIntakeControl(
    Controls.operatorController::getRightTriggerAxis,
    Controls.operatorController::getLeftTriggerAxis
  );
  private final ResetGyroHeading resetGyroHeading = new ResetGyroHeading();
  private final DefaultPosition defaultPosition = new DefaultPosition();

  /* Global Robot State */
  private final SendableChooser<Command> autoChooser;
  public static boolean isOperatorOverride = false;

  /* Triggers */
  public static final Trigger doNotScore = Controls.operatorController.leftTrigger(Controls.AXIS_DEADBAND);

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

    // Set default command for lights
    lightsSubsystem.setDefaultCommand(new DefaultLightCommand());

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
    // Use the back button to zero both elevator and pivot in sequence
    Controls.operatorController.back().onTrue(new ZeroMechanisms());
    

    // Normal mode button bindings
    inNormalMode.and(Controls.operatorController.a()).onTrue(
      new InstantCommand(() -> { dynamicPathingSubsystem.setCoralScoringLevel(ScoringLevel.L1); })
    );
    inNormalMode.and(Controls.operatorController.x()).onTrue(
      new InstantCommand(() -> { dynamicPathingSubsystem.setCoralScoringLevel(ScoringLevel.L2); })
    );
    inNormalMode.and(Controls.operatorController.b()).onTrue(
      new InstantCommand(() -> { dynamicPathingSubsystem.setCoralScoringLevel(ScoringLevel.L3); })
    );
    inNormalMode.and(Controls.operatorController.y()).onTrue(
      new InstantCommand(() -> { dynamicPathingSubsystem.setCoralScoringLevel(ScoringLevel.L4); })
    );

    // SysID routines
    // sysIDBindings();
    
    // Override mode immediately moves to position while held
    inOverrideMode.and(Controls.operatorController.a()).whileTrue(
      Commands.either(
        Commands.parallel(
          new SetElevatorPos(ElevatorLevel.L1),
          new SetPivotPos(PivotPosition.L1)
        ), 
        Commands.parallel(
          new SetElevatorPos(ElevatorLevel.PROCESSOR),
          new SetPivotPos(PivotPosition.PROCESSOR)
        ), 
        () -> intakeSubsystem.isCoralLoaded()
      )
    ).onFalse(defaultPosition);

    inOverrideMode.and(Controls.operatorController.x()).whileTrue(
      Commands.either(
        Commands.parallel(
          new SetElevatorPos(ElevatorLevel.L2),
          new SetPivotPos(PivotPosition.L2)
        ), 
        Commands.parallel(
          new SetElevatorPos(ElevatorLevel.ALGAE_L1),
          new SetPivotPos(PivotPosition.ALGAE_L1)
        ), 
        () -> intakeSubsystem.isCoralLoaded()
      )
    ).onFalse(defaultPosition);

    inOverrideMode.and(Controls.operatorController.b()).whileTrue(
      Commands.either(
        Commands.parallel(
          new SetElevatorPos(ElevatorLevel.L3),
          new SetPivotPos(PivotPosition.L3)
        ), 
        Commands.parallel(
          new SetElevatorPos(ElevatorLevel.ALGAE_L2),
          new SetPivotPos(PivotPosition.ALGAE_L2)
        ), 
        () -> intakeSubsystem.isCoralLoaded()
      )
    ).onFalse(defaultPosition);

    inOverrideMode.and(Controls.operatorController.y()).whileTrue(
      Commands.either(
        Commands.parallel(
          new SetElevatorPos(ElevatorLevel.L4),
          new SetPivotPos(PivotPosition.L4)
        ), 
        Commands.parallel(
          new SetElevatorPos(ElevatorLevel.NET),
          new SetPivotPos(PivotPosition.NET)
        ), 
        () -> intakeSubsystem.isCoralLoaded()
      )
    ).onFalse(defaultPosition);


    // Axis intake control 
    intakeSubsystem.setDefaultCommand(axisIntakeControl);

    // Manual auto intake
    Controls.operatorController.leftBumper().whileTrue(
      Commands.parallel(
        new CoralIntake(),
        new ApplyScoringSetpoint(ScoringLevel.CORAL_INTAKE)
      )    
    );

    // Operator Algea out
    Controls.operatorController.rightBumper().whileTrue(
      new AlgeaOutake()
    );

    // Dynamic pathing button
    Controls.dynamicPathingButton.whileTrue(
      Commands.defer(
        () -> dynamicPathingSubsystem.getCurrentDynamicPathCommand(), 
        new HashSet<>(Arrays.asList(driveSubsystem))
      )
      // On start - Begin rumble and start dynamic pathing
      .beforeStarting(() -> Controls.operatorController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.5))
      // On end - Stop rumble
      .finallyDo(() -> Controls.operatorController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0))
    );

    // Switch coral scoring side
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
    
    // Additional L3/R3 controls for coral scoring side which are also the back paddles
    Controls.operatorController.rightStick().onTrue(
      new InstantCommand(
        () -> {dynamicPathingSubsystem.setCoralScoringSide(true);}
      )
    );
    Controls.operatorController.leftStick().onTrue(
      new InstantCommand(
        () -> {dynamicPathingSubsystem.setCoralScoringSide(false);}
      )
    );
  }

  private static void toggleOperatorOverride() {
    isOperatorOverride = !isOperatorOverride;
    telemetry.publishOperatorOverrideInfo();
  }

  /** Binds controls to run drivetrain sysID */
  private void sysIDBindings() {
    Controls.operatorController.a().whileTrue(
     driveSubsystem.sysIdQuasistatic(Direction.kForward)
    );
    Controls.operatorController.x().whileTrue(
      driveSubsystem.sysIdQuasistatic(Direction.kReverse)
    );
    Controls.operatorController.b().whileTrue(
      driveSubsystem.sysIdDynamic(Direction.kForward)
    );
    Controls.operatorController.y().whileTrue(
      driveSubsystem.sysIdDynamic(Direction.kReverse)
    );
    Controls.operatorController.leftBumper().onTrue(
      new InstantCommand(() -> {SignalLogger.start(); System.out.println("LOG START");})  
    );
    Controls.operatorController.rightBumper().onTrue(
      new InstantCommand(() -> {SignalLogger.stop(); System.out.println("LOG STOP");})  
    );
  }

  /**
   * Use this method to define name->command mappings. Names will be used by PathPlanner to 
   * call commands in full autos. 
   */
  private void registerNamedCommands() {
    // Register Named Commands
    // Add other commands to be able to run them in autos
    // NamedCommands.registerCommand("exampleCommand", exampleCommand);
    var scoringCommandRequirements = new HashSet<>(Arrays.asList(driveSubsystem, pivotSubsystem, elevatorSubsystem, intakeSubsystem));
    
    // L4 Right
    NamedCommands.registerCommand("Autoscore L4 Right", Commands.defer(
      () -> ScoreCoral.scoreCoralWithSettings(ScoringLevel.L4, true), scoringCommandRequirements)
    );

    // L4 Left
    NamedCommands.registerCommand("Autoscore L4 Left", Commands.defer(
      () -> ScoreCoral.scoreCoralWithSettings(ScoringLevel.L4, false), scoringCommandRequirements)
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /**
   * Gets the elevator subsystem instance.
   * @return The elevator subsystem
   */
  public Elevator getElevator() {
    return elevatorSubsystem;
  }

  /**
   * Gets the drive subsystem instance.
   * @return The drive subsystem
   */
  public DriveSubsystem getDriveSubsystem() {
    return driveSubsystem;
  }
}
