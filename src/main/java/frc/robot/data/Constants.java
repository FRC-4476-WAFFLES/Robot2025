// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.data;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

  /* CAN IDs  */

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
  public static final int climberLeader = 11;
  public static final int climberFollower = 12;
  public static final int climberAllignment = 13;
  public static final int coralIntakeMotor = 14;
  public static final int coralPivotMotor = 15;
  public static final int algaeIntakeMotor = 16;
  public static final int algaePivotMotor = 17;
  public static final int funnelMotor = 18;
  
  // Sensors
  
  public static final int frontLeftAbsoluteEncoder = 19; // CANcoder
  public static final int frontRightAbsoluteEncoder = 20; // CANcoder
  public static final int backLeftAbsoluteEncoder = 21; // CANcoder
  public static final int backRightAbsoluteEncoder = 22; // CANcoder
  public static final int climberAbsoluteEncoder = 23;
  public static final int climberEncoderOffset = 0;
  public static final int coralPivotAbsoluteEncoder = 24;
  public static final int coralPivotAbsoluteEncoderOffset = 0;
  
  public static final int pidgeon = 25;

  public static final int CANdle = 26; // CAN ID for the CANdle device

  /* PWM Outputs */
  
  public static final int addressableLEDS = 2; //Light Strip
  public static final int lightsBlinkin = 3; // REV Blinkin

  /* Digital Ports */
  
  public static final int coastModeSwitch = 4; // Limit Switch

  /* Vision */
  public static class VisionConstants {
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 1);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  /* Physical */
  public static class PhysicalConstants {
    // Placeholder values. Tune.
    public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double maxAngularSpeed = 6; // Max Rad/s
  }
}
