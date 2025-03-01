// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.data;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import static edu.wpi.first.units.Units.MetersPerSecond;
import frc.robot.data.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.data.Constants.ManipulatorConstants.PivotPosition;

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

  /* Code */
  public static class CodeConstants {
    public static final int SUBSYSTEM_NT_UPDATE_RATE = 20; // How many times a second subsystems will publish to NT. Reduce if performance is suffering.
  }

  /* Vision */
  public static class VisionConstants {
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(2, 2, 1);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public static final Matrix<N3, N1> kSingleTagStdDevsMT1 = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, 5);
    public static final Matrix<N3, N1> kMultiTagStdDevsMT1 = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, 3);

    public static final Matrix<N3, N1> kStdDevsMT2ReefTargeting = VecBuilder.fill(0.05,0.05, Double.MAX_VALUE);
    public static final Matrix<N3, N1> kStdDevsMT2 = VecBuilder.fill(0.25,0.25, Double.MAX_VALUE);

    // Reject mt1 poses if further than this from current estimate, removes ambiguity noise
    // Only to be used when not seeding position
    public static final double MT1_REJECT_DISTANCE = 2; // meters

    public static final int SEDING_LL_IMU_MODE = 1; // Enables seeding
    public static final int MOVING_LL_IMU_MODE = 2; // Should be 3, but firmware isn't out yet, might use 2 if feeling like it

    // Names of limelights
    public static final String LIMELIGHT_NAME_L = "limelight-right";
    public static final String LIMELIGHT_NAME_R = "limelight-left"; 

    // Exclusively rely on reef tags for megatag
    // Prob want to expand this a lot later, but for terminal guidance this is all we should care about
    public static final int[] RED_VALID_REEF_TAG_IDs = {
      6, 7, 8, 9, 10, 11 
    };

    public static final int[] BLUE_VALID_REEF_TAG_IDs = {
      17, 18, 19, 20, 21, 22  
    };
  }

  /* Physical */
  public static class PhysicalConstants {
    // Placeholder values. Tune.
    public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double maxAngularSpeed = 6; // Max Rad/s

    public static final double withBumperBotHalfWidth = 0.460; // m

    // In number of motor rotations per mechanism rotation
    public static final double funnelReduction = 40.0; 
    public static final double pivotReduction = 52.5625; 
    public static final double ClimberReduction = 341.3333333;
    public static final double elevatorReductionToMeters = 52.1602684; // Motor rotations to elevator height in meters

    public static final double pivotAbsoluteEncoderOffset = 0;
  }

  /* Funnel Constants */
  public static class FunnelConstants {
    // Control constants
    public static final double FUNNEL_DEAD_ZONE = 1.0; // In degrees
    public static final double FUNNEL_MIN_ANGLE = 0.0; // Minimum angle in degrees
    public static final double FUNNEL_MAX_ANGLE = 200.0; // Maximum angle in degrees - adjust as needed
    public static final double FUNNEL_BLOCKING_THRESHOLD = 45.0; // Angle threshold where funnel starts to interfere with elevator movement

    // Motor configuration
    public static final double STATOR_CURRENT_LIMIT = 40.0; // amps
    public static final double MOTION_CRUISE_VELOCITY = 110.0; 
    public static final double MOTION_ACCELERATION = 190.0; 
    public static final double MOTION_JERK = 1900.0; 

    // PID Values
    public static final double kP = 2.0;
    public static final double kI = 0.0;
    public static final double kD = 0.01;
    public static final double kS = 0.0;

    // Predefined positions for the funnel (in degrees)
    public enum FunnelPosition {
      DOWN(0.0),
      UP(200.0);

      private final double degrees;

      FunnelPosition(double degrees) {
        this.degrees = degrees;
      }

      public double getDegrees() {
        return degrees;
      }
    }
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
      CORAL_INTAKE(PivotPosition.CORAL_INTAKE, ElevatorLevel.REST_POSITION),
      PROCESSOR(PivotPosition.PROCESSOR, ElevatorLevel.PROCESSOR),
      NET(PivotPosition.NET, ElevatorLevel.NET);
      
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
    public static final double ALGAE_CURRENT_THRESHOLD = 30.0; // amps

    // Intake constantsd
    public static final double INTAKE_SPEED_MULTIPLIER = 0.15;
    public static final double INTAKE_MAX_SPEED = 25; // Rps
    public static final double ALGAE_HOLD_SPEED = 2; // Speed to hold algae in place

    // Pivot constants
    public static final double PIVOT_ANGLE_DEADBAND = 1.3;
    public static final double PIVOT_MIN_ANGLE = 0.0; // degrees
    public static final double PIVOT_MAX_ANGLE = 185.0; // degrees
    public static final double PIVOT_BUMPER_CLEARANCE_ANGLE = 144.4; // degrees
    public static final double PIVOT_L4_CLEARANCE_ANGLE = 28;

    // Motor configuration
    public static final double PIVOT_MOTION_CRUISE_VELOCITY = 4.0;
    public static final double PIVOT_MOTION_ACCELERATION = 20.0;
    public static final double PIVOT_MOTION_JERK = 2000.0;
    public static final double STATOR_CURRENT_LIMIT = 45.0; // amps
    public static final double PIVOT_MOTOR_DEADBAND = 0.002;
    public static final double PIVOT_CURRENT_THRESHOLD =  14.0; // amps - Current threshold for zeroing

    // PID Values
    public static final double PIVOT_kP = 110.0;
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
        NET(64),
        L4(49),
        L3(28),
        L2(35),
        L1(150),

        // Maybe manual mode
        MANUAL_L4(71.0),
        MANUAL_L3(24.0),
        MANUAL_L2(24.0),
        MANUAL_L1(0.0);

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
    public static final double ELEVATOR_DEAD_ZONE = 0.015;
    public static final double ZEROING_SPEED = -0.1; // Slow downward speed
    public static final double STALL_CURRENT_THRESHOLD = 20.0; // Amperes

    // Elevator will not move if the pivot is not past this angle, to avoid collision with top bar
    public static final double MIN_ELEVATOR_PIVOT_ANGLE = 20; 
    public static final double PIVOT_BUMPER_CLEAR_HEIGHT = 0.181;

    public static final double PIVOT_L4_CLEAR_HEIGHT_MIN = 0.70;
    public static final double PIVOT_L4_CLEAR_HEIGHT_MAX = 1.2;

    public static final double MIN_ELEVATOR_HEIGHT = 0;
    public static final double MAX_ELEVATOR_HEIGHT = 1.47;

    // Collision zone constants
    public static final double COLLISION_ZONE_LOWER = 0.12; // meters
    public static final double COLLISION_ZONE_UPPER = 0.56; // meters

    // Height where first stage starts moving
    public static final double FIRST_STAGE_START_HEIGHT = ElevatorConstants.MAX_ELEVATOR_HEIGHT / 2.0; 

    // Motion Magic configuration
    public static final double MOTION_CRUISE_VELOCITY = 4; // 4 usually
    public static final double MOTION_ACCELERATION = 8;
    public static final double MOTION_JERK = 2000;

    // PID Values
    public static final double kP = 85.0;
    public static final double kI = 0.0;
    public static final double kD = 0.01;
    public static final double kS = 0.0;
    public static final double kG = 0.36; // Volts of feedforward. Not put in kG gain slot since only applies when elevator is past constant force springs

    // Predefined heights for the elevator (in meters)
    public enum ElevatorLevel {
      REST_POSITION(0.0),
      NET(1.45),
      ALGAE_L2(0.89),
      ALGAE_L1(0.55),
      PROCESSOR(0.2),
      L4(1.47),
      L3(0.772),
      L2(0.41),
      L1(0.33),

      MANUAL_L4(1.440),
      MANUAL_L3(0.6772),
      MANUAL_L2(0.280),
      MANUAL_L1(0.22); 


      private final double height;

      ElevatorLevel(double height) {
        this.height = height;
      }

      public double getHeight() {
        return height;
      }
    }
  }

  /* Climber Constants */
  public static class ClimberConstants {
    // Control constants
    public static final double CLIMBER_DEAD_ZONE = 1.0; // In degrees
    public static final double CLIMBER_MIN_ANGLE = 0.0; // Minimum angle in degrees
    public static final double CLIMBER_MAX_ANGLE = 30.0; // Maximum angle in degrees

    // Motor configuration
    public static final double STATOR_CURRENT_LIMIT = 60.0; // amps
    public static final double MOTION_CRUISE_VELOCITY = 110.0; 
    public static final double MOTION_ACCELERATION = 190.0; 
    public static final double MOTION_JERK = 1900.0; 

    // PID Values
    public static final double kP = 2.0;
    public static final double kI = 0.0;
    public static final double kD = 0.01;
    public static final double kS = 0.0;

    // Predefined positions for the climber (in degrees)
    public enum ClimberPosition {
      RETRACTED(0.0),
      DEPLOYED(30.0);

      private final double degrees;

      ClimberPosition(double degrees) {
        this.degrees = degrees;
      }

      public double getDegrees() {
        return degrees;
      }
    }
  }
}
