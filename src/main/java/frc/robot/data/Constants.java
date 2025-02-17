// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.data;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import frc.robot.data.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.data.Constants.ManipulatorConstants.PivotPosition;

import static edu.wpi.first.units.Units.MetersPerSecond;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /* CAN IDs  */
  public static class CANIds {
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
    public static final int funnelMotor = 13;
    public static final int intakeMotor = 14;
    public static final int pivotMotor = 15;
    
    
    // Sensors
    
    public static final int frontLeftAbsoluteEncoder = 19; // CANcoder
    public static final int frontRightAbsoluteEncoder = 20; // CANcoder
    public static final int backLeftAbsoluteEncoder = 21; // CANcoder
    public static final int backRightAbsoluteEncoder = 22; // CANcoder
    // public static final int climberAbsoluteEncoder = 23;
    // public static final int climberEncoderOffset = 0;
    // public static final int coralPivotAbsoluteEncoder = 24;
    // public static final int coralPivotAbsoluteEncoderOffset = 0;

    public static final int pivotAbsoluteEncoder = 28;
    
    public static final int laserCan = 29;

    public static final int pidgeon = 25;

    public static final int CANdle = 26;
  }

  /* PWM Outputs */
  public static class PWMOutputs {
    // we should have none
  }

  /* Digital Ports */
  public static class DigitalOutputs {
    public static final int coastModeSwitch = 4; // Limit Switch
  }

  /* Vision */
  public static class VisionConstants {
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(2, 2, 1);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public static final Matrix<N3, N1> kSingleTagStdDevsMT1 = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, 5);
    public static final Matrix<N3, N1> kMultiTagStdDevsMT1 = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, 3);

    public static final Matrix<N3, N1> kStdDevsMT2 = VecBuilder.fill(1,1, Double.MAX_VALUE);

    // Reject mt1 poses if further than this from current estimate, removes ambiguity noise
    // Only to be used when not seeding position
    public static final double MT1_REJECT_DISTANCE = 2; // meters

    public static final int DISABLED_LL_IMU_MODE = 1; // Enables seeding
    public static final int ENABLED_LL_IMU_MODE = 2; // Should be 3, but firmware isn't out yet, might use 2 if feeling like it
  }

  /* Physical */
  public static class PhysicalConstants {
    // Placeholder values. Tune.
    public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double maxAngularSpeed = 6; // Max Rad/s

    public static final double withBumperBotHalfWidth = 0.460; // m

    // In number of motor rotations per mechanism rotation
    public static final double funnelReduction = 33.75; 
    public static final double pivotReduction = 52.5625; 
    public static final double ClimberReduction = -1; // We don't have a climber yet lol
    public static final double elevatorReductionToMeters = 52.1602684; // Motor rotations to elevator height in meters

    public static final double pivotAbsoluteEncoderOffset = 0;
  }

  public static class ScoringConstants {
    public static final double AUTO_SCORE_PIVOT_NUDGE = 8;
    /**
     * Maps scoring levels to their respective elevator and pivot enums
     */
    public enum ScoringLevel {
      L1(PivotPosition.L1, ElevatorLevel.L1),
      L2(PivotPosition.L2, ElevatorLevel.L2),
      L3(PivotPosition.L3, ElevatorLevel.L3),
      L4(PivotPosition.L4, ElevatorLevel.L4),
      ALGEA_L1(PivotPosition.ALGAE_L1, ElevatorLevel.ALGAE_L1),
      ALGEA_L2(PivotPosition.ALGAE_L2, ElevatorLevel.ALGAE_L2),
      CORAL_INTAKE(PivotPosition.CORAL_INTAKE, ElevatorLevel.REST_POSITION);
      
      private final PivotPosition pivotPosition;
      private final ElevatorLevel elevatorLevel;

      ScoringLevel(PivotPosition pivotPosition, ElevatorLevel elevatorLevel) {
          this.pivotPosition = pivotPosition;
          this.elevatorLevel = elevatorLevel;
      }

      public ElevatorLevel getElevatorLevel() {
          return elevatorLevel;
      }

      public PivotPosition getPivotPosition() {
          return pivotPosition;
      }
    }
  }

  /* Manipulator Constants */
  public static class ManipulatorConstants {
    // Detection thresholds
    public static final double CORAL_LOADED_DISTANCE_THRESHOLD = 22.0; // mm
    public static final double ALGAE_LOADED_DISTANCE_THRESHOLD = 94.0; // mm
    public static final double ALGAE_LOADED_DISTANCE_UPPER_LIMIT = 130; // mm
    public static final double ALGAE_CURRENT_THRESHOLD = 34.0; // amps

    // Intake constants
    public static final double INTAKE_SPEED_MULTIPLIER = 0.15;
    public static final double INTAKE_MAX_SPEED = 25; // Rps
    public static final double ALGAE_HOLD_SPEED = 4; // Speed to hold algae in place

    // Pivot constants
    public static final double PIVOT_ANGLE_DEADBAND = 1;
    public static final double PIVOT_MIN_ANGLE = 0.0; // degrees
    public static final double PIVOT_MAX_ANGLE = 185.0; // degrees
    public static final double PIVOT_BUMPER_CLEARANCE_ANGLE = 144.4; // degrees
    public static final double PIVOT_L4_CLEARANCE_ANGLE = 28;

    // Motor configuration
    public static final double PIVOT_MOTION_CRUISE_VELOCITY = 4.0;
    public static final double PIVOT_MOTION_ACCELERATION = 20.0;
    public static final double PIVOT_MOTION_JERK = 2000.0;
    public static final double STATOR_CURRENT_LIMIT = 60.0; // amps
    public static final double PIVOT_MOTOR_DEADBAND = 0.002;
    public static final double PIVOT_CURRENT_THRESHOLD = 6.0; // amps - Current threshold for zeroing

    // PID Values
    public static final double PIVOT_kP = 100.0;
    public static final double PIVOT_kI = 0.0;
    public static final double PIVOT_kD = 0.0;
    public static final double PIVOT_kS = 0.0;

    public static final double PIVOT_kP_ALGEA_SLOW = 40.0;

    // Pivot Positions
    public enum PivotPosition {
        ZERO(0),
        CLEARANCE_POSITION(22),
        CLEARANCE_POSITION_ALGEA(90),
        ALGAE_L2(169.1),
        ALGAE_L1(178.5),
        PROCESSOR(189),
        CORAL_INTAKE(0.5),
        NET(10),
        L4(51.8),
        L3(45.0),
        L2(45.0),
        L1(13.0);

        private final double pivotDegrees;

        PivotPosition(double pivotDegrees) {
            this.pivotDegrees = pivotDegrees;
        }

        public double getDegrees() {
            return pivotDegrees;
        }
    }
  }

  /* Elevator Constants */
  public static class ElevatorConstants {
    // Control constants
    public static final double ELEVATOR_DEAD_ZONE = 1;
    public static final double ZEROING_SPEED = -0.1; // Slow downward speed
    public static final double STALL_CURRENT_THRESHOLD = 20.0; // Amperes

    // Elevator will not move if the pivot is not past this angle, to avoid collision with top bar
    public static final double MIN_ELEVATOR_PIVOT_ANGLE = 20; 
    public static final double PIVOT_BUMPER_CLEAR_HEIGHT = 0.181;

    public static final double PIVOT_L4_CLEAR_HEIGHT_MIN = 0.74;
    public static final double PIVOT_L4_CLEAR_HEIGHT_MAX = 1.1;

    public static final double MIN_ELEVATOR_HEIGHT = 0;
    public static final double MAX_ELEVATOR_HEIGHT = 1.46;

    // Collision zone constants
    public static final double COLLISION_ZONE_LOWER = 0.12; // meters
    public static final double COLLISION_ZONE_UPPER = 0.84; // meters

    // Motion Magic configuration
    public static final double MOTION_CRUISE_VELOCITY = 4; // 4 usually
    public static final double MOTION_ACCELERATION = 8;
    public static final double MOTION_JERK = 2000;

    // PID Values
    public static final double kP = 85.0;
    public static final double kI = 0.0;
    public static final double kD = 0.01;
    public static final double kS = 0.0;
    public static final double kG = 0.18; // Volts of feedforward was 0.36 but halved with constant force springs

    // Predefined heights for the elevator (in meters)
    public enum ElevatorLevel {
      REST_POSITION(0.0),
      NET(1.0),
      ALGAE_L2(1.0),
      ALGAE_L1(0.472),
      PROCESSOR(0.2),
      L4(1.400),
      L3(0.7376),
      L2(0.36),
      L1(0.118); 


      private final double height;

      ElevatorLevel(double height) {
        this.height = height;
      }

      public double getHeight() {
        return height;
      }
    }
  }
}
