// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.data.Constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public final TalonFX Elevator1;
    private final TalonFX Elevator2;
    public boolean isClimbing = false;
   
    private double elevatorTargetPosition = 0;
    private boolean isZeroingElevator = false;

    private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    private static final double ELEVATOR_DEAD_ZONE = 1;

    private static final CurrentLimitsConfigs elevatorCurrentLimits = new CurrentLimitsConfigs();

    private static final double ZEROING_SPEED = -0.1; // Slow downward speed
    private static final double STALL_CURRENT_THRESHOLD = 10.0; // Amperes

    public enum elevatorLevel {
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
    Elevator2.setControl(new Follower(Constants.elevator1, true));

    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    elevatorCurrentLimits.StatorCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits = elevatorCurrentLimits;
    elevatorConfig.CurrentLimits = elevatorCurrentLimits;

    // set slot 0 gains
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0; // Keeping the existing value
    slot0Configs.kP = 2; // Keeping the existing value
    slot0Configs.kI = 0; // Keeping the existing value
    slot0Configs.kD = 0.01; // Keeping the existing value

    elevatorConfig.Slot0 = slot0Configs;

    // Configure MotionMagic
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = 110; // Using the existing velocity value
    motionMagicConfigs.MotionMagicAcceleration = 190; // Using the existing acceleration value
    motionMagicConfigs.MotionMagicJerk = 1900; // Setting jerk to 10x acceleration as a starting point
    elevatorConfig.MotionMagic = motionMagicConfigs;

    Elevator1.getConfigurator().apply(elevatorConfig);
    Elevator2.getConfigurator().apply(elevatorConfig);
  }
    /**
   * Periodic method called by the command scheduler.
   * Updates elevator state, manages profiling, and updates SmartDashboard.
   */
  @Override
  public void periodic() {
    executeElevatorMotionMagic();

    if (Elevator1.getStatorCurrent().getValueAsDouble() < STALL_CURRENT_THRESHOLD && isZeroingElevator) {
      // Stop the elevator
      Elevator1.set(0);

      // Set the current position as the new zero
      Elevator1.setPosition(0);

      // Reset the target position
      elevatorTargetPosition = 0;

      // Update the current level
      currentLevel = elevatorLevel.L0;

      isZeroingElevator = false;

      System.out.println("Elevator zeroed successfully");
    }
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
    elevatorTargetPosition = position;
  }

  public double getElevatorPosition(){
    return Elevator1.getPosition().getValueAsDouble();
  }

  /**
   * Gets the target position of the elevator in rotations.
   * @return The target position of the elevator in rotations.
   */
  public double getElevatorTargetPosition(){
    return elevatorTargetPosition;
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

  public void setLevel0(){
    currentLevel = elevatorLevel.L0;
  }

  public void setLevel3() {
    currentLevel = elevatorLevel.L3;
  }

  public void setLevel2() {
    currentLevel = elevatorLevel.L2;
  }

  public void setLevel1() {
    currentLevel = elevatorLevel.L1;
  }




  public void zeroElevator() {
    // Drive elevator down slowly
    Elevator1.set(ZEROING_SPEED);
    isZeroingElevator = true;

    // // Wait until the elevator stalls (hits the base)
    // while (Elevator1.getStatorCurrent().getValueAsDouble() < STALL_CURRENT_THRESHOLD) {
    //     // Small delay to prevent tight looping
    //     try {
    //         Thread.sleep(20);
    //     } catch (InterruptedException e) {
    //         e.printStackTrace();
    //     }
    // }
    //}
}
  /**
   * Gets the current elevator mode.
   * @return The current ShooterMode of the elevator.
   */
  public elevatorLevel getElevatorMode(){
    return currentLevel;
  }


}
