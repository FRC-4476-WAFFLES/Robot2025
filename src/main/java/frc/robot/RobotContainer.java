// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.ResetGyroHeading;
import frc.robot.commands.intake.AlgaeOutake;
import frc.robot.commands.intake.AutoIntake;
import frc.robot.commands.intake.AxisIntakeControl;
import frc.robot.commands.intake.CoralIntake;
import frc.robot.commands.scoring.ScoreCoral;
import frc.robot.commands.scoring.ScoreNet;
import frc.robot.commands.shark.SharkCommands;
import frc.robot.commands.superstructure.ApplyScoringSetpoint;
import frc.robot.commands.superstructure.SetElevatorPos;
import frc.robot.commands.superstructure.SetPivotPos;
import frc.robot.commands.superstructure.SuperstructureControl;
import frc.robot.commands.superstructure.ZeroMechanisms;
import frc.robot.commands.test.TestDriveAuto;
import frc.robot.commands.test.TestElevatorAuto;
import frc.robot.commands.test.WheelRadiusCharacterization;
import frc.robot.data.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.data.Constants.ManipulatorConstants.PivotPosition;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.data.Constants.ScoringConstants;
import frc.robot.data.Constants.ScoringConstants.ScoringLevel;
import frc.robot.data.TunerConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DynamicPathing;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.MechanismPoses;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.SharkIntake;
import frc.robot.subsystems.SharkPivot;
import frc.robot.subsystems.Telemetry;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  /* Hardware Subsystems */
  public static final DriveSubsystem driveSubsystem = TunerConstants.createDrivetrain();
  public static final Pivot pivotSubsystem = new Pivot();
  public static final Intake intakeSubsystem = new Intake();
  public static final Elevator elevatorSubsystem = new Elevator();
  public static final Lights lightsSubsystem = new Lights();
  public static final SharkIntake sharkIntake = new SharkIntake();
  public static final SharkPivot sharkPivot = new SharkPivot();

  /* Software Subsystems */
  /* Do not control harware, but have state and or periodic methods */
  /* Can be required by commands to mutex lock actions like pathing */
  public static final DynamicPathing dynamicPathingSubsystem = new DynamicPathing();
  public static final Telemetry telemetry = new Telemetry(PhysicalConstants.maxSpeed);
  public static final MechanismPoses mechanismPoses = new MechanismPoses();

  /* Commands */
  private final Command resetGyroHeading = new ResetGyroHeading().ignoringDisable(true);
  private final Command restPosition = SuperstructureControl.RestPositionCommand();
  private final Command axisIntakeControl = new AxisIntakeControl(
    Controls.operatorController::getRightTriggerAxis,
    Controls.operatorController::getLeftTriggerAxis
  );

  /* Global Robot State */
  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<Command> testChooser;
  public static boolean isOperatorOverride = false;
  public static boolean isRunningL1Intake = false;

  public static Trigger isHeadingLockedToL1;

  /** The static entry point for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure trigger bindings
    configureBindings();

    // Swerve telemetry from odometry thread
    driveSubsystem.registerTelemetry(telemetry::telemeterize);
    driveSubsystem.setDefaultCommand(new DriveTeleop(
      Controls::getDriveY,
      Controls::getDriveX,
      Controls::getDriveRotation
    ));

    // Axis intake control 
    intakeSubsystem.setDefaultCommand(axisIntakeControl);

    // Default superstructure commands
    pivotSubsystem.setDefaultCommand(SuperstructureControl.PivotDefaultCommand());
    elevatorSubsystem.setDefaultCommand(SuperstructureControl.ElevatorDefaultCommand());

    // Register commands to be used by pathplanner autos
    registerNamedCommands();

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    testChooser = buildTestChooser(); 
    SmartDashboard.putData("Test Routine Chooser", testChooser);

    // Warmup pathplanner to reduce delay when dynamic pathing
    FollowPathCommand.warmupCommand().schedule();
  }

  /**
   * Binds controls
   */
  private void configureBindings() {
    // Create conditional triggers based on operator override state
    Trigger inNormalMode = new Trigger(() -> !isOperatorOverride);
    Trigger inOverrideMode = new Trigger(() -> isOperatorOverride);

    Trigger runningL1Intake = new Trigger(() -> isRunningL1Intake);
    Trigger sharkCoralLoaded = new Trigger(() -> sharkIntake.isCoralLoaded());

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

    // Manual auto intake
    Controls.operatorController.leftBumper().whileTrue(
      Commands.parallel(
        new CoralIntake(),
        new ApplyScoringSetpoint(ScoringLevel.CORAL_INTAKE)
      )    
    );

    // Operator Algea out
    dynamicPathingSubsystem.notRunningAction.and(Controls.algaeOut).whileTrue(
      new SequentialCommandGroup(
        new InstantCommand(() -> RobotContainer.pivotSubsystem.setIsThrowingAlgae(true)),
        new ParallelRaceGroup(
          new ApplyScoringSetpoint(ScoringLevel.SPIT_ALGAE),
          new WaitCommand(0.45) // wait some amount of time ¯\_(ツ)_/¯
        ),
        new AlgaeOutake()
      ).finallyDo(() -> {
        RobotContainer.pivotSubsystem.setIsThrowingAlgae(false);
      })
    ).onFalse(restPosition);
    
    // Override mode immediately moves to position while held
    inOverrideMode.and(Controls.operatorController.a()).whileTrue(
      Commands.either(
        Commands.parallel(
          new SetElevatorPos(ElevatorLevel.MANUAL_L1),
          new SetPivotPos(PivotPosition.MANUAL_L1)
        ), 
        Commands.parallel(
          new SetElevatorPos(ElevatorLevel.PROCESSOR),
          new SetPivotPos(PivotPosition.PROCESSOR)
        ), 
        () -> intakeSubsystem.isCoralLoaded()
      )
    ).onFalse(restPosition);

    inOverrideMode.and(Controls.operatorController.x()).whileTrue(
      Commands.either(
        Commands.parallel(
          new SetElevatorPos(ElevatorLevel.MANUAL_L2),
          new SetPivotPos(PivotPosition.MANUAL_L2)
        ), 
        Commands.parallel(
          new SetElevatorPos(ElevatorLevel.ALGAE_L1),
          new SetPivotPos(PivotPosition.ALGAE_L1)
        ), 
        () -> intakeSubsystem.isCoralLoaded()
      )
    ).onFalse(restPosition);

    inOverrideMode.and(Controls.operatorController.b()).whileTrue(
      Commands.either(
        Commands.parallel(
          new SetElevatorPos(ElevatorLevel.MANUAL_L3),
          new SetPivotPos(PivotPosition.MANUAL_L3)
        ), 
        Commands.parallel(
          new SetElevatorPos(ElevatorLevel.ALGAE_L2),
          new SetPivotPos(PivotPosition.ALGAE_L2)
        ), 
        () -> intakeSubsystem.isCoralLoaded()
      )
    ).onFalse(restPosition);

    inOverrideMode.and(Controls.operatorController.y()).whileTrue(
      Commands.either(
        Commands.parallel(
          new SetElevatorPos(ElevatorLevel.MANUAL_L4),
          new SetPivotPos(PivotPosition.MANUAL_L4)
        ), 
        Commands.parallel(
          new SetElevatorPos(ElevatorLevel.NET),
          new SetPivotPos(PivotPosition.NET)
        ), 
        () -> intakeSubsystem.isCoralLoaded()
      )
    ).onFalse(restPosition);

    // Dynamic pathing button
    Controls.dynamicPathingButton.whileTrue(
      Commands.defer(
        () -> dynamicPathingSubsystem.getCurrentDynamicActionCommand(), 
        ScoreCoral.commandRequirements
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

    // Manual net toss
    Controls.operatorController.povDown().whileTrue(Commands.defer(() -> ScoreNet.getScoreNetCommand(0, Rotation2d.kZero, false), ScoreCoral.commandRequirements).onlyIf(() -> RobotContainer.intakeSubsystem.isAlgaeLoaded()));
  
    // L1 Intake / Outtake
    Controls.rightJoystick.button(4).onTrue(
      Commands.either(
        Commands.runOnce(() -> RobotContainer.isRunningL1Intake = !RobotContainer.isRunningL1Intake), 
        SharkCommands.getOutakeCommand().asProxy(), 
        () -> !sharkIntake.isCoralLoaded()
      )
    );

    // Run intake while intake should be running lmao
    runningL1Intake.whileTrue(SharkCommands.getIntakeCommand());
    
    // Heading lock for L1
    isHeadingLockedToL1 = sharkCoralLoaded.and(() -> 
      DynamicPathing.isRobotInRangeOfReefL1() && 
      dynamicPathingSubsystem.notRunningAction.getAsBoolean() && 
      Controls.getDriveRotationRaw() < ScoringConstants.L1_HEADING_LOCK_RIPOFF_VALUE &&
      Math.abs(driveSubsystem.getRobotPose().getRotation().minus(dynamicPathingSubsystem.getClosestFaceAngle()).getDegrees()) < ScoringConstants.L1_HEADING_LOCK_ENGAGE_DIFFERENCE
    );

    isHeadingLockedToL1.whileTrue(
      new DriveTeleop(
        Controls::getDriveY, false,
        Controls::getDriveX, false,
        () -> dynamicPathingSubsystem.getClosestFaceAngle(), true        
      )
    );
    
    // Manual intake backup
    Controls.leftJoystick.button(2).onTrue(
      new InstantCommand(() -> {
        intakeSubsystem.setTargetPosition(intakeSubsystem.getCurrentPosition() + 1);
      })
    );
    // Controls.operatorController.rightBumper().whileTrue(
    //   AutoIntake.GetAutoIntakeCommand()  
    // );


  }

  /**
   * Toggles operator override mode, and updates it's value on networktables
   */
  private static void toggleOperatorOverride() {
    isOperatorOverride = !isOperatorOverride;
    telemetry.publishOperatorOverrideInfo();
  }

  /** Binds controls to run drivetrain sysID */
  private void sysIDBindings() {
    // Drive bindings
    Controls.operatorController.a().whileTrue(
     elevatorSubsystem.m_sysIdRoutineElevator.quasistatic(Direction.kForward)
    );
    Controls.operatorController.x().whileTrue(
      elevatorSubsystem.m_sysIdRoutineElevator.quasistatic(Direction.kReverse)
    );
    Controls.operatorController.b().whileTrue(
      elevatorSubsystem.m_sysIdRoutineElevator.dynamic(Direction.kForward)
    );
    Controls.operatorController.y().whileTrue(
      elevatorSubsystem.m_sysIdRoutineElevator.dynamic(Direction.kReverse)
    );

    // Datalog controls needed by sysID
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
    
    
    // Direct position commands for both elevator and pivot
    NamedCommands.registerCommand("Set Position L1", Commands.parallel(
      new SetElevatorPos(ElevatorLevel.L1),
      new SetPivotPos(PivotPosition.L1)
    ));
    // SCUFFED
    NamedCommands.registerCommand("Set Position L2", 
      Commands.deadline(
        SuperstructureControl.L4ScorePrepCommand(),
        new CoralIntake().onlyIf(() -> !intakeSubsystem.isCoralLoaded())
      )
      );

    NamedCommands.registerCommand("Set Position L3", Commands.parallel(
      new SetElevatorPos(ElevatorLevel.L3),
      new SetPivotPos(PivotPosition.L3)
    ));
    NamedCommands.registerCommand("Set Position L4", Commands.parallel(
      new SetElevatorPos(ElevatorLevel.L4),
      new SetPivotPos(PivotPosition.L4)
    ));
    NamedCommands.registerCommand("Set Position Processor", Commands.parallel(
      new SetElevatorPos(ElevatorLevel.PROCESSOR),
      new SetPivotPos(PivotPosition.PROCESSOR)
    ));
    NamedCommands.registerCommand("Set Position Net", Commands.parallel(
      new SetElevatorPos(ElevatorLevel.NET),
      new SetPivotPos(PivotPosition.NET)
    ));
    NamedCommands.registerCommand("Set Position Algae L1", Commands.parallel(
      new SetElevatorPos(ElevatorLevel.ALGAE_L1),
      new SetPivotPos(PivotPosition.ALGAE_L1)
    ));
    NamedCommands.registerCommand("Set Position Algae L2", Commands.parallel(
      new SetElevatorPos(ElevatorLevel.ALGAE_L2),
      new SetPivotPos(PivotPosition.ALGAE_L2)
    ));
    
    // L4 
    NamedCommands.registerCommand("Autoscore L4 Right", Commands.defer(
      () -> ScoreCoral.scoreCoralWithSettings(ScoringLevel.L4, true), ScoreCoral.commandRequirements)
    );
    NamedCommands.registerCommand("Autoscore L4 Left", Commands.defer(
      () -> ScoreCoral.scoreCoralWithSettings(ScoringLevel.L4, false), ScoreCoral.commandRequirements)
    );

    // L3
    NamedCommands.registerCommand("Autoscore L3 Right", Commands.defer(
      () -> ScoreCoral.scoreCoralWithSettings(ScoringLevel.L3, true), ScoreCoral.commandRequirements)
    );
    NamedCommands.registerCommand("Autoscore L3 Left", Commands.defer(
      () -> ScoreCoral.scoreCoralWithSettings(ScoringLevel.L3, false), ScoreCoral.commandRequirements)
    );

    // L2
    NamedCommands.registerCommand("Autoscore L2 Right", Commands.defer(
      () -> ScoreCoral.scoreCoralWithSettings(ScoringLevel.L2, true), ScoreCoral.commandRequirements)
    );
    NamedCommands.registerCommand("Autoscore L2 Left", Commands.defer(
      () -> ScoreCoral.scoreCoralWithSettings(ScoringLevel.L2, false), ScoreCoral.commandRequirements)
    );

    // L1
    NamedCommands.registerCommand("Autoscore L1 Right", Commands.defer(
      () -> ScoreCoral.scoreCoralWithSettings(ScoringLevel.L1, true), ScoreCoral.commandRequirements)
    );
    NamedCommands.registerCommand("Autoscore L1 Left", Commands.defer(
      () -> ScoreCoral.scoreCoralWithSettings(ScoringLevel.L1, false), ScoreCoral.commandRequirements)
    );

    // Coral Intake
    NamedCommands.registerCommand("Coral Intake", Commands.parallel(
      new CoralIntake(),
      new ApplyScoringSetpoint(ScoringLevel.CORAL_INTAKE)
    ));

    NamedCommands.registerCommand("Set Position Intake", 
      Commands.sequence(
        new WaitUntilCommand(() -> DynamicPathing.isElevatorRetractionSafe()),
        new ApplyScoringSetpoint(ScoringLevel.CORAL_INTAKE)
      )
    );

    

    // Auto Coral Intake
    NamedCommands.registerCommand("Auto Coral Intake", AutoIntake.GetAutoIntakeCommand());
  }

  /**
   * Use this method to define a list of commands that can be chosen from in test mode
   */
  private SendableChooser<Command> buildTestChooser() {
    SendableChooser<Command> chooser = new SendableChooser<>();

    chooser.setDefaultOption("None", Commands.none());
    chooser.addOption("Wheel Radius Characterization", WheelRadiusCharacterization.GetCharacterizationCommand());
    chooser.addOption("Test Drivetrain", new TestDriveAuto(driveSubsystem));
    chooser.addOption("Test Elevator", new TestElevatorAuto(elevatorSubsystem));

    return chooser;
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
   * Use this to pass the testing command to the main {@link Robot} class.
   *
   * @return the command to run in testng mode
   */
  public Command getTestCommand() {
    return testChooser.getSelected();
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
