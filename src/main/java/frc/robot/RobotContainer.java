// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ClimberIn;
import frc.robot.commands.ClimberOut;
import frc.robot.commands.DriveTeleop;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.Funnel;

import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DynamicPathingSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.utils.Telemetry;

import java.util.Arrays;
import java.util.HashSet;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeCoral;
import frc.robot.data.TunerConstants;
import frc.robot.data.Constants.PhysicalConstants;
import frc.robot.subsystems.ElevatorSubsystem;
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
  public static final DynamicPathingSubsystem dynamicPathingSubsystem = new DynamicPathingSubsystem();
  public static final CoralManipulator coralIntake = new CoralManipulator();
  public static final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public static final CoralManipulator coralPivot = new CoralManipulator();
  public static final Funnel funnel = new Funnel();
  public static final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public static final AlgaeManipulator algeManipulator = new AlgaeManipulator();


  /* Commands */
  private final IntakeCoral runIntake = new IntakeCoral();
  //final ElevatorCommand setElevatorLevel = new ElevatorCommand();
  private final ClimberIn climberIn = new ClimberIn();
  private final ClimberOut climberOut = new ClimberOut();


  /* Global Robot State */
  private final Telemetry telemetry = new Telemetry(PhysicalConstants.maxSpeed);
  private final SendableChooser<Command> autoChooser;
  public boolean isOperatorOverride = false;

  /** The static entry point for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Swerve telemetry from odometry thread
    driveSubsystem.registerTelemetry(telemetry::telemeterize);
    driveSubsystem.setDefaultCommand(new DriveTeleop(
      Controls::getDriveX,
      Controls::getDriveY,
      Controls::getDriveRotation
    ));

    // Register commands to be used by pathplanner autos
    registerNamedCommands();

    // Publish build info once to networktables
    telemetry.publishBuildInfo();

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    Controls.operatorController.leftTrigger().onTrue(
      new InstantCommand(
        () -> {isOperatorOverride = !isOperatorOverride;}
      )
    );
    //Controls.operatorController.a().onTrue(elevator command for setting L0);
    //Controls.operatorController.b().onTrue(elevator command for setting L1);
    //Controls.operatorController.x().onTrue(elevator command for setting L2);
    //Controls.operatorController.y().onTrue(elevator command for setting L3);
    Controls.operatorController.povUp().whileTrue(runIntake);
    //Controls.operatorController.povDown().whileTrue(runOuttake);
    Controls.operatorController.rightStick().onTrue(climberIn);
    Controls.operatorController.leftStick().onTrue(climberOut);
    //Controls.operatorController.leftBumper().onTrue(pivot L0 command);
    //Controls.operatorController.rightBumper().onTrue(pivot L1-2 command);
    //Controls.operatorController.rightTrigger().onTrue(pivot command for L3); 

    // Switch coral scoring sides IF OVERRIDE IS NOT ON
    if (isOperatorOverride == false) {
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
    // Dynamic path to coral scoring
    Controls.leftJoystick.button(1).whileTrue(Commands.defer(
      () -> dynamicPathingSubsystem.getCoralPathCommand(), new HashSet<>(Arrays.asList(driveSubsystem))
    ));
    }
    if (isOperatorOverride == true){
      // elevatorSubsystem.adjustTargetPosition(rightAxis);
    }
    

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
