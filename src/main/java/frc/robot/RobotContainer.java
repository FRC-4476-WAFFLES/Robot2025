// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.autos.OPP2Lolipop;
import frc.robot.autos.OPP2Post2345;
import frc.robot.autos.US2Lolipop;
import frc.robot.autos.US2Post98710;
import frc.robot.commands.ResetGyroHeading;
import frc.robot.commands.drive.DriveTeleop;
import frc.robot.commands.intake.AlgaeOutake;
import frc.robot.commands.intake.AxisIntakeControl;
import frc.robot.commands.intake.CoralOutake;
import frc.robot.commands.scoring.ScoreCoral;
import frc.robot.commands.scoring.ScoreNet;
import frc.robot.commands.superstructure.ApplySuperstructureState;
import frc.robot.commands.superstructure.ExecuteHandoff;
import frc.robot.commands.superstructure.GroundAlgaePickup;
import frc.robot.commands.superstructure.SuperstructureControl;
import frc.robot.commands.superstructure.ZeroMechanisms;
import frc.robot.commands.test.TestDriveAuto;
import frc.robot.commands.test.TestElevatorAuto;
import frc.robot.commands.test.WheelRadiusCharacterization;
import frc.robot.data.Constants.CodeConstants;
import frc.robot.data.Constants.ScoringConstants;
import frc.robot.data.Constants.VisionConstants;
import frc.robot.data.TunerConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DynamicPathing;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.MechanismPoses;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.groundsuperstructure.GroundIntakeSuperstructure;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.utils.vision.LimelightHelpers;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  /* Global Robot State */
  private SendableChooser<Command> autoChooser;
  private SendableChooser<Command> testChooser;
  public static boolean isOperatorOverride = false;
  public static boolean isRunningL1Intake = false;
  public static boolean isGroundIntakingAlgae = false;
  public static Trigger isHeadingLockedToL1;

  /* Hardware Subsystems */
  public static final DriveSubsystem driveSubsystem = TunerConstants.createDrivetrain();
  public static final Superstructure superstructure = new Superstructure(); // Contains two other subsystems
  public static final GroundIntakeSuperstructure groundSuperstructure = new GroundIntakeSuperstructure(); // Contains two other subsystems
  public static final Intake intakeSubsystem = new Intake();
  public static final Lights lightsSubsystem = new Lights();


  /* Software Subsystems */
  /* Do not control harware, but have state and or periodic methods */
  /* Can be required by commands to mutex lock actions like pathing */
  public static final DynamicPathing dynamicPathingSubsystem = new DynamicPathing();
  public static final Telemetry telemetry = new Telemetry();
  public static final MechanismPoses mechanismPoses = new MechanismPoses();

  /* Commands */
  private final Command resetGyroHeading = new ResetGyroHeading().ignoringDisable(true);
  private final Command restPosition = SuperstructureControl.RestPositionCommand();
  private final Command axisIntakeControl = new AxisIntakeControl(
    Controls.operatorController::getRightTriggerAxis,
    Controls.operatorController::getLeftTriggerAxis
  );

  Trigger triggerHandoff;

  /** The static entry point for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure trigger bindings
    configureBindings();

    // Swerve telemetry from odometry thread
    driveSubsystem.registerTelemetry(telemetry::telemetryConsumer);
    driveSubsystem.setDefaultCommand(new DriveTeleop(
      Controls::getDriveY,
      Controls::getDriveX,
      Controls::getDriveRotation
    ));

    // Axis intake control 
    intakeSubsystem.setDefaultCommand(axisIntakeControl);

    // Default superstructure commands
    superstructure.pivot.setDefaultCommand(SuperstructureControl.PivotDefaultCommand());
    superstructure.elevator.setDefaultCommand(SuperstructureControl.ElevatorDefaultCommand());

    // Register commands to be used by pathplanner autos
    registerNamedCommands();

    // Build an auto chooser. This will use Commands.none() as the default option.
    if (CodeConstants.USE_PATHPLANNER_AUTOS) {
      autoChooser = AutoBuilder.buildAutoChooser();
    } else {
      autoChooser = new SendableChooser<>();
      autoChooser.addOption("OPP2 Lolipop", new OPP2Lolipop());
      autoChooser.addOption("OPP2 2,3,4,5", new OPP2Post2345());
      autoChooser.addOption("US2 Lolipop", new US2Lolipop());
      autoChooser.addOption("US2 9,8,7,10", new US2Post98710());
    }
    
    SmartDashboard.putData("Auto Chooser", autoChooser);

    testChooser = buildTestChooser(); 
    SmartDashboard.putData("Test Routine Chooser", testChooser);

    // Warmup pathplanner to reduce delay when dynamic pathing
    FollowPathCommand.warmupCommand().schedule();

    // Configure coral tracker
    LimelightHelpers.setPipelineIndex(VisionConstants.LIMELIGHT_NAME_CORAL, 1);
  }

  /**
   * Binds controls
   */
  private void configureBindings() {
    // Create conditional triggers based on operator override state
    Trigger inNormalMode = new Trigger(() -> !isOperatorOverride);
    Trigger inOverrideMode = new Trigger(() -> isOperatorOverride);

    Trigger algaeGroundIntakeActive = new Trigger(() -> isGroundIntakingAlgae);

    Trigger L1Loaded = new Trigger(() -> groundSuperstructure.isL1Ready());
    triggerHandoff = new Trigger(() -> groundSuperstructure.isHandoffReady() && !intakeSubsystem.isAlgaeLoaded() && !intakeSubsystem.isCoralLoaded());

    // Toggle operator override
    Controls.operatorController.start().onTrue(
      new InstantCommand(RobotContainer::toggleOperatorOverride)
    );

    Controls.rightJoystick.button(9).onTrue(resetGyroHeading);
    // Use the back button to zero both elevator and pivot in sequence
    Controls.operatorController.back().onTrue(new ZeroMechanisms());
    

    // Normal mode button bindings
    // inNormalMode.and(Controls.operatorController.a()).onTrue(
    //   new InstantCommand(() -> { dynamicPathingSubsystem.setCoralScoringLevel(SuperstructureState.L1); })
    // );
    inNormalMode.and(Controls.operatorController.x()).onTrue(
      new InstantCommand(() -> { dynamicPathingSubsystem.setCoralScoringLevel(SuperstructureState.L2); })
    );
    inNormalMode.and(Controls.operatorController.b()).onTrue(
      new InstantCommand(() -> { dynamicPathingSubsystem.setCoralScoringLevel(SuperstructureState.L3); })
    );
    inNormalMode.and(Controls.operatorController.y()).onTrue(
      new InstantCommand(() -> { dynamicPathingSubsystem.setCoralScoringLevel(SuperstructureState.L4); })
    );

    // SysID routines
    // sysIDBindings();

    // Intake
    inNormalMode.and(Controls.rightJoystick.button(4)).onTrue(
      Commands.runOnce(() -> groundSuperstructure.handoffIntakeToggle())
    );

    inNormalMode.and(Controls.leftJoystick.button(3)).onTrue(
      Commands.runOnce(() -> groundSuperstructure.L1IntakeToggle())
    );

    // Controls.driverController.y().onTrue(
    //   Commands.runOnce(() -> isGroundIntakingAlgae = !isGroundIntakingAlgae)
    // );

    algaeGroundIntakeActive.whileTrue(new GroundAlgaePickup());

    // Operator Algea out
    dynamicPathingSubsystem.notRunningAction.and(Controls.algaeOut).whileTrue(
      new SequentialCommandGroup(
        new InstantCommand(() -> RobotContainer.superstructure.pivot.setIsThrowingAlgae(true)),
        new ParallelCommandGroup(
          new ApplySuperstructureState(SuperstructureState.SPIT_ALGAE)
        ),
        new AlgaeOutake()
      ).finallyDo(() -> {
        RobotContainer.superstructure.pivot.setIsThrowingAlgae(false);
      })
    ).onFalse(restPosition);
    
    // Override mode immediately moves to position while held
    inOverrideMode.and(Controls.operatorController.a()).whileTrue(
      Commands.either(
        new ApplySuperstructureState(SuperstructureState.MANUAL_L1),
        new ApplySuperstructureState(SuperstructureState.PROCESSOR), 
        () -> intakeSubsystem.isCoralLoaded()
      )
    ).onFalse(restPosition);

    inOverrideMode.and(Controls.operatorController.x()).whileTrue(
      Commands.either(
        new ApplySuperstructureState(SuperstructureState.MANUAL_L2), 
        new ApplySuperstructureState(SuperstructureState.ALGAE_L1),
        () -> intakeSubsystem.isCoralLoaded()
      )
    ).onFalse(restPosition);

    inOverrideMode.and(Controls.operatorController.b()).whileTrue(
      Commands.either(
        new ApplySuperstructureState(SuperstructureState.MANUAL_L3), 
        new ApplySuperstructureState(SuperstructureState.ALGAE_L2),
        () -> intakeSubsystem.isCoralLoaded()
      )
    ).onFalse(restPosition);

    inOverrideMode.and(Controls.operatorController.y()).whileTrue(
      Commands.either(
        new ApplySuperstructureState(SuperstructureState.MANUAL_L4), 
        new ApplySuperstructureState(SuperstructureState.NET),
        () -> intakeSubsystem.isCoralLoaded()
      )
    ).onFalse(restPosition);

    // Dynamic pathing button
    Controls.dynamicPathingButton.whileTrue(
      Commands.defer(
        () -> dynamicPathingSubsystem.getCurrentDynamicActionCommand(), 
        DynamicPathing.actionCommandRequirements
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

    // Controls.operatorController.povUp().whileTrue(
    //   new ParallelCommandGroup(
    //     new InstantCommand(
    //       () -> {intakeSubsystem.setIntakeSpeed(ManipulatorConstants.ALGAE_INTAKE_SPEED);}
    //     ),
    //     new ApplySuperstructureState(SuperstructureState.GROUND_PICKUP_ALGAE)
    //   )
    // );

    // Manual net toss
    Controls.operatorController.povDown().whileTrue(Commands.defer(() -> ScoreNet.getScoreNetCommand(0, () -> Rotation2d.kZero, false, true), DynamicPathing.actionCommandRequirements).onlyIf(() -> RobotContainer.intakeSubsystem.isAlgaeLoaded()));
    
    // Heading lock for L1
    isHeadingLockedToL1 = L1Loaded.and(() -> 
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

    // Only trigger handoff here in teleop
    triggerHandoff.and(() -> DriverStation.isTeleop()).onTrue(new ExecuteHandoff());

    // Simulation
    if (RobotBase.isSimulation()) { 
      Controls.simController.button(1).onTrue(
        Commands.runOnce(() -> telemetry.toggleIntakeSimLoaded())
      );
        
      Controls.simController.button(2).onTrue(
        Commands.runOnce(() -> telemetry.toggleIntakeHandoffSimLoaded())
      );
        
      Controls.simController.button(3).onTrue(
        Commands.runOnce(() -> telemetry.toggleManipulatorCoralSimLoaded())
      );

      Controls.simController.button(4).onTrue(
        Commands.runOnce(() -> telemetry.toggleAlgeaSimLoaded())
      );
    }
  }

  /**
   * Toggles operator override mode, and updates it's value on networktables
   */
  private static void toggleOperatorOverride() {
    isOperatorOverride = !isOperatorOverride;
    telemetry.publishOperatorOverrideInfo();
  }

  /** Binds controls to run drivetrain sysID */
  @SuppressWarnings("unused")
  private void sysIDBindings() {
    // Drive bindings
    Controls.operatorController.a().whileTrue(
     superstructure.elevator.m_sysIdRoutineElevator.quasistatic(Direction.kForward)
    );
    Controls.operatorController.x().whileTrue(
      superstructure.elevator.m_sysIdRoutineElevator.quasistatic(Direction.kReverse)
    );
    Controls.operatorController.b().whileTrue(
      superstructure.elevator.m_sysIdRoutineElevator.dynamic(Direction.kForward)
    );
    Controls.operatorController.y().whileTrue(
      superstructure.elevator.m_sysIdRoutineElevator.dynamic(Direction.kReverse)
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
    
    // Sends the elevator up in stages in preparation for L4 score
    // Name is legacy that isn't worth changing in pathplanner at this point
    NamedCommands.registerCommand("Set Position L2", 
      // Commands.deadline(
        SuperstructureControl.L4ScorePrepCommand()
      // )
      // Commands.runOnce(() -> {
      //   RobotContainer.superstructure.elevator.applySetpoint(SuperstructureState.HANDOFF_CLEAR);
      //   RobotContainer.superstructure.pivot.applySetpoint(SuperstructureState.HANDOFF_CLEAR);
      // })
    );

    // Direct position commands for both elevator and pivot
    NamedCommands.registerCommand("Set Position L1", Commands.parallel(
      new ApplySuperstructureState(SuperstructureState.L1)
    ));
    NamedCommands.registerCommand("Set Position L3", Commands.parallel(
      new ApplySuperstructureState(SuperstructureState.L3)
    ));
    NamedCommands.registerCommand("Set Position L4", Commands.parallel(
      new ApplySuperstructureState(SuperstructureState.L4)
    ));
    NamedCommands.registerCommand("Set Position Processor", Commands.parallel(
      new ApplySuperstructureState(SuperstructureState.PROCESSOR)
    ));
    NamedCommands.registerCommand("Set Position Net", Commands.parallel(
      new ApplySuperstructureState(SuperstructureState.NET)
    ));
    NamedCommands.registerCommand("Set Position Algae L1", Commands.parallel(
      new ApplySuperstructureState(SuperstructureState.ALGAE_L1)
    ));
    NamedCommands.registerCommand("Set Position Algae L2", Commands.parallel(
      new ApplySuperstructureState(SuperstructureState.ALGAE_L2)
    ));

    // Auto score commands
    
    // L4 
    NamedCommands.registerCommand("Autoscore L4 Right", Commands.defer(
      () -> ScoreCoral.scoreCoralWithSettings(SuperstructureState.L4, true), DynamicPathing.actionCommandRequirements)
    );
    NamedCommands.registerCommand("Autoscore L4 Left", Commands.defer(
      () -> ScoreCoral.scoreCoralWithSettings(SuperstructureState.L4, false), DynamicPathing.actionCommandRequirements)
    );

    // L3
    NamedCommands.registerCommand("Autoscore L3 Right", Commands.defer(
      () -> ScoreCoral.scoreCoralWithSettings(SuperstructureState.L3, true), DynamicPathing.actionCommandRequirements)
    );
    NamedCommands.registerCommand("Autoscore L3 Left", Commands.defer(
      () -> ScoreCoral.scoreCoralWithSettings(SuperstructureState.L3, false), DynamicPathing.actionCommandRequirements)
    );

    // L2
    NamedCommands.registerCommand("Autoscore L2 Right", Commands.defer(
      () -> ScoreCoral.scoreCoralWithSettings(SuperstructureState.L2, true), DynamicPathing.actionCommandRequirements)
    );
    NamedCommands.registerCommand("Autoscore L2 Left", Commands.defer(
      () -> ScoreCoral.scoreCoralWithSettings(SuperstructureState.L2, false), DynamicPathing.actionCommandRequirements)
    );

    // Coral Intake
    // NamedCommands.registerCommand("Coral Intake", Commands.parallel(
    //   new CoralIntake(),
    //   new ApplySuperstructureState(SuperstructureState.CORAL_INTAKE)
    // ));

    NamedCommands.registerCommand("Set Position Intake", 
      Commands.sequence(
        Commands.parallel(
          new CoralOutake(),
          Commands.runOnce(() -> groundSuperstructure.handoffIntakeToggle())
        ).withTimeout(0.5),
        new WaitUntilCommand(() -> DynamicPathing.isElevatorRetractionSafe()),      
        Commands.runOnce(() -> superstructure.applySuperstructureState(SuperstructureState.HANDOFF_READY)),
        Commands.waitUntil(() -> triggerHandoff.getAsBoolean()),
        new ExecuteHandoff()
      )
    );

    NamedCommands.registerCommand("Lolipop Intake", 
      Commands.sequence(
        new WaitUntilCommand(() -> RobotContainer.intakeSubsystem.isCoralLoaded())
      )
    );

    NamedCommands.registerCommand("Auto Coral Intake", 
      Commands.sequence(
        // new AlignToCoral(null, null, null),
        new WaitUntilCommand(() -> RobotContainer.groundSuperstructure.isHandoffHappening())
      )
    );

    // Algae manipulation
    NamedCommands.registerCommand("Auto Algae Intake", Commands.defer(
      () -> dynamicPathingSubsystem.createAlgaePickupCommand(), DynamicPathing.actionCommandRequirements
    ));

    NamedCommands.registerCommand("Net Shot", Commands.defer(
      () -> dynamicPathingSubsystem.createScoreNetCommand(), DynamicPathing.actionCommandRequirements
    ));

    NamedCommands.registerCommand("Set Coral Loaded", 
      Commands.runOnce(() -> RobotContainer.intakeSubsystem.forceLoadCoral())
    );

    NamedCommands.registerCommand("Net Shot Prep", Commands.parallel(
      Commands.runOnce(() -> superstructure.elevator.applySetpoint(SuperstructureState.NET_PREP))
    ));
  }

  /**
   * Use this method to define a list of commands that can be chosen from in test mode
   */
  private SendableChooser<Command> buildTestChooser() {
    SendableChooser<Command> chooser = new SendableChooser<>();

    chooser.setDefaultOption("None", Commands.none());
    chooser.addOption("Wheel Radius Characterization", WheelRadiusCharacterization.GetCharacterizationCommand());
    chooser.addOption("Test Drivetrain", new TestDriveAuto(driveSubsystem));
    chooser.addOption("Test Elevator", new TestElevatorAuto(superstructure.elevator));

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
}
