// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
import edu.wpi.first.math.util.Units;
public final class Constants {
  public static final int intakeMotor=5;
  public static final int alignmentMotor = 16;


  // CAN IDs

  // Drive Motors
  public static final int steeringFrontLeft = 1; 
  public static final int drivingFrontLeft = 2; 
  public static final int steeringFrontRight = 3; 
  public static final int drivingFrontRight = 4; 
  public static final int steeringBackLeft = 5; 
  public static final int drivingBackLeft = 6; 
  public static final int steeringBackRight = 7; 
  public static final int drivingBackRight = 8; 

  // Other Motors
  public static final int elevator1 = 9; // Kraken X60
  public static final int elevator2 = 10; // Kraken X60
  public static final int climberLeader = 18;
  public static final int climberFollower = 17;
  public static final int climberAllignment=16;
  public static final int coralGrabMotor=11;
  public static final int coralPivotMotor=12;
  public static final int algaeIntakeMotor=13;
  public static final int algaePivotMotor=14;
  public static final int coralIntakeMotor=15;
  
  // Sensors
  
  public static final int frontLeftAbsoluteEncoder = 11; // CANcoder
  public static final int frontRightAbsoluteEncoder = 12; // CANcoder
  public static final int backLeftAbsoluteEncoder = 13; // CANcoder
  public static final int backRightAbsoluteEncoder = 14; // CANcoder
  public static final int climberAbsoluteEncoder=19;
  public static final int pivotAbsoluteEncoder=20;
  public static final int climberEncoderOffset=0;
  public static final int pivotAbsoluteEncoderOffset=0;
  public static final int pidgeon = 15;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int leftJoystick = 1;
    public static final int rightJoystick = 2;
  }
  public static final int CANdle = 1; // CAN ID for the CANdle device

  // PWM outputs
  public static final int addressableLEDS = 2; //Light Strip
  public static final int lightsBlinkin = 3; // REV Blinkin
// Digital input
public static final int coastModeSwitch = 4; // Limit Switch
  
}
