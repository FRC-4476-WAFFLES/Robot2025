// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

import frc.robot.Constants;
public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public final TalonFX Elevator1;
    private final TalonFX Elevator2;

    public boolean isClimbing = false;

    private final DigitalInput coastSwitch;
   
    private double elevatorTargetPosition = 0;

    private final double ELEVATOR_DEAD_ZONE = 1;

    private final CurrentLimitsConfigs elevatorCurrentLimits = new CurrentLimitsConfigs();

    private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    private Timer profileTimer = new Timer();
    private boolean previousEnabled = false;

    private double previousTargetPosition = elevatorTargetPosition;
    private double profileStartPosition = 0;
    private double profileStartVelocity = 0;

    private boolean previousSwitchState;

    public enum elevatorLevel {
      NET(90),
      ALGAE_L2(70),
      ALGAE_L1(60),
      PROCESSOR(55),
      CORAL_INTAKE(40),
      L3(50.0),
      L2(27.0),
      L1(10.0),
      L0(2);

      private double height;
  
      elevatorLevel(double height) {
        this.height = height;
      }
  
      public double getHeight() {
        return height;
      }
    }
    
    elevatorLevel currentLevel = elevatorLevel.L0;
  public ElevatorSubsystem() {
    Elevator1 = new TalonFX(Constants.elevator1);
    Elevator2 = new TalonFX(Constants.elevator2);
    coastSwitch = new DigitalInput(Constants.coastModeSwitch);
    Elevator2.setControl(new Follower(Constants.elevator1, true));

    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    elevatorCurrentLimits.SupplyCurrentLimit = 40;
    elevatorCurrentLimits.SupplyTimeThreshold  = 40;
    elevatorCurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorCurrentLimits.StatorCurrentLimit = 40;
    elevatorCurrentLimits.StatorCurrentLimitEnable = true;

    elevatorConfig.CurrentLimits = elevatorCurrentLimits;
    
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0; // Keeping the existing value
    slot0Configs.kP = 2; // Keeping the existing value
    slot0Configs.kI = 0; // Keeping the existing value
    slot0Configs.kD = 0.01; // Keeping the existing value

    elevatorConfig.Slot0 = slot0Configs;

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = 110; // Using the existing velocity value
    motionMagicConfigs.MotionMagicAcceleration = 190; // Using the existing acceleration value
    motionMagicConfigs.MotionMagicJerk = 1900; // Setting jerk to 10x acceleration as a starting point
    elevatorConfig.MotionMagic = motionMagicConfigs;

    Elevator1.getConfigurator().apply(elevatorConfig);
    Elevator2.getConfigurator().apply(elevatorConfig);

    profileTimer.start();
  }

public void periodic() {
 if(DriverStation.isDisabled()){
      this.profileStartPosition = this.Elevator1.getPosition().getValueAsDouble();
    }
    executeElevatorMotionMagic();

    if (!getCoastSwitch() && previousSwitchState){
      Elevator1.setNeutralMode(NeutralModeValue.Coast);
      Elevator2.setNeutralMode(NeutralModeValue.Coast);
    }
    else if(getCoastSwitch() && !previousSwitchState){
      Elevator1.setNeutralMode(NeutralModeValue.Brake);
      Elevator2.setNeutralMode(NeutralModeValue.Brake);
    }
    previousSwitchState = getCoastSwitch(); 
}

  /**
   * Executes motion profiling for the elevator.
   */
  private void executeElevatorMotionMagic() {
    motionMagicRequest.Position = elevatorTargetPosition;
    motionMagicRequest.Slot = 0; // Use the Slot0 gains
    Elevator1.setControl(motionMagicRequest);
  }

   /**
   * Sets the target position of the elevator.
   * @param position Target position in rotations.
   */
  public void setElevatorTargetPosition(double position){
    this.elevatorTargetPosition = position;
    if(this.elevatorTargetPosition != this.previousTargetPosition){
      profileTimer.restart();
      this.previousTargetPosition = this.elevatorTargetPosition;
      this.profileStartPosition = this.Elevator1.getPosition().getValueAsDouble();
      this.profileStartVelocity = this.Elevator1.getVelocity().getValueAsDouble();
    }
  }

  /**
   * Gets the current elevator position in meters.
   * @return The current elevator position in meters.
   */
  public double getElevatorPositionMeters(){
    return rotationsToInches(Elevator1.getPosition().getValueAsDouble())*0.0254;
  }
  /**
   * Converts rotations to inches.
   * @param rotations Number of rotations.
   * @return Equivalent distance in inches.
   */
  public double rotationsToInches(double rotations){
   return (rotations/19.0625)*(1.625*Math.PI);
  }

  /**
   * Converts inches to rotations.
   * @param inches Distance in inches.
   * @return Equivalent number of rotations.
   */
  public double inchesToRotations (double inches){
    return (inches*19.0625)/(1.625/Math.PI);
  }
  /**
   * Checks if the elevator is at the desired position.
   * @return true if elevator is at desired position, false otherwise.
   */
  public boolean isGoodElevatorPosition() {
    return Math.abs(Elevator1.getPosition().getValueAsDouble() - elevatorTargetPosition) < ELEVATOR_DEAD_ZONE;
  }

  /**
   * Adjusts the target position of the elevator.
   * @param change The amount to adjust the target position by.
   */
  public void adjustTargetPosition(double change) {
    elevatorTargetPosition += change;
  }

  /**
   * Gets the state of the coast mode switch.
   * @return The state of the coast mode switch.
   */
  public boolean getCoastSwitch(){
    return coastSwitch.get();
  }

  /**
   * Sets the shooter mode to bottom.
   */
  public void setBL0(){
    currentShooterMode = ShooterMode.L0;
  }

  /**
   * Sets the shooter mode to tall.
   */
  public void setL3() {
    currentShooterMode = ShooterMode.L3;
  }

  /**
   * Sets the shooter mode to middle.
   */
  public void setL2() {
    currentShooterMode = ShooterMode.L2;
  }

  /**
   * Sets the shooter mode to short.
   */
  public void setNet() {
    currentShooterMode = ShooterMode.NET;
  }
  
  public void setAlgaeL2() {
    currentShooterMode = ShooterMode.ALGAE_L2;
  }
  
  public void setAlgaeL1() {
    currentShooterMode = ShooterMode.ALGAE_l1;
  }
  
  public void setProcessor() {
    currentShooterMode = ShooterMode.PROCESSOR;
  }

  public void setCoralIntake() {
    currentShooterMode = ShooterMode.CORAL_INTAKE;
  }
  /**
   * Gets the current elevator mode.
   * @return The current ShooterMode of the elevator.
   */
  public ShooterMode getElevatorMode(){
    return currentShooterMode;
  }
}
