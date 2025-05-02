// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.data;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
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
    public static final int elevator1 = 9; 
    public static final int elevator2 = 10; 
    public static final int intakeMotor = 14;
    public static final int pivotMotor = 15;
    public static final int sharkPivotMotor = 12;
    public static final int sharkIntakeMotor = 13;
    
    // Sensors
    
    public static final int frontLeftAbsoluteEncoder = 19; // CANcoder
    public static final int frontRightAbsoluteEncoder = 20; // CANcoder
    public static final int backLeftAbsoluteEncoder = 21; // CANcoder
    public static final int backRightAbsoluteEncoder = 22; // CANcoder
    // public static final int coralPivotAbsoluteEncoder = 24;
    // public static final int coralPivotAbsoluteEncoderOffset = 0;

    public static final int pivotAbsoluteEncoder = 28;
    
    public static final int intakeLaserCan = 29;
    public static final int funnelLaserCan = 30;

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
    public static final int coralSensor = 9; // Coral detection sensor
  }

  /* Code */
  public static class CodeConstants {
    public static final int SUBSYSTEM_NT_UPDATE_RATE = 20; // How many times a second subsystems will publish to NT. Reduce if performance is suffering.
    public static final boolean FORCE_LOAD_SIM_CORAL = false;
    public static final boolean FORCE_LOAD_SIM_ALGAE = false;
  }

  /* Vision */
  public static class VisionConstants {
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(2, 2, 1);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public static final Matrix<N3, N1> kSingleTagStdDevsMT1 = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, 4);
    public static final Matrix<N3, N1> kMultiTagStdDevsMT1 = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, 3);

    public static final Matrix<N3, N1> kStdDevsMT2ReefTargeting = VecBuilder.fill(0.01,0.01, Double.MAX_VALUE);
    public static final Matrix<N3, N1> kStdDevsMT2 = VecBuilder.fill(0.25,0.25, Double.MAX_VALUE);

    // Reject mt1 poses if further than this from current estimate, removes ambiguity noise
    // Only to be used when not seeding position
    public static final double MT1_REJECT_DISTANCE = 2; // meters

    public static final int SEDING_LL_IMU_MODE = 1; // Enables seeding
    public static final int MOVING_LL_IMU_MODE = 2; // Uses internal IMU

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

  /* Field */
  public static class FieldConstants {
    // Viewed from blue alliance driver station 
    public static final Translation2d HumanPlayerLeftPos = new Translation2d(1.6, 7.35);
    public static final Translation2d HumanPlayerRightPos = new Translation2d(1.6, 0.7); 
  }

  /* Physical */
  public static class PhysicalConstants {
    // Placeholder values. Tune.
    public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double maxAngularSpeed = 6; // Max Rad/s

    public static final double withBumperBotHalfWidth = 0.460; // m

    // In number of motor rotations per mechanism rotation
    public static final double sharkIntakeReduction = 9.0; 
    public static final double sharkPivotReduction = 37.92592592592592592; 
    public static final double pivotReduction = 52.5625; 
    public static final double intakeReduction = 2.5;

    public static final double elevatorReductionToMeters = 26.6; // Motor rotations to elevator height in meters

    public static final double pivotAbsoluteEncoderOffset = -0.267822265625;
    public static final boolean usePivotAbsoluteEncoder = true; // Fallback, if false relies on internal motor encoder
  }

  public static class ScoringConstants {
    public static final double AUTO_SCORE_PIVOT_NUDGE = 8;
    public static final double ALGAE_TOSS_PIVOT_ANGLE = 155; // Angle at which toss occurs
    public static final double L1_HEADING_LOCK_ENGAGE_DIFFERENCE = 45; // If within 30deg of L1 angle, engage heading lock 
    public static final double L1_HEADING_LOCK_RIPOFF_VALUE = 0.2; // Break auto align if joystick value higher than this

    // Makes the elevator go up more in net autos, we can tip over but it *is* faster! :)
    public static final boolean USE_RISKY_NET_AUTO = true;
    /**
     * Maps scoring levels to their respective elevator and pivot enums
     */
    public enum ScoringLevel {
      L1(PivotPosition.L1, ElevatorLevel.L1),
      L2(PivotPosition.L2, ElevatorLevel.L2),
      L3(PivotPosition.L3, ElevatorLevel.L3),
      L4(PivotPosition.L4, ElevatorLevel.L4),
      ALGAE_L1(PivotPosition.ALGAE_L1, ElevatorLevel.ALGAE_L1),
      ALGAE_L2(PivotPosition.ALGAE_L2, ElevatorLevel.ALGAE_L2),
      CORAL_INTAKE(PivotPosition.CORAL_INTAKE, ElevatorLevel.REST_POSITION),
      PROCESSOR(PivotPosition.PROCESSOR, ElevatorLevel.PROCESSOR),
      SPIT_ALGAE(PivotPosition.SPIT_ALGAE, ElevatorLevel.PROCESSOR),
      NET(PivotPosition.NET, ElevatorLevel.NET),
      NET_PREP(PivotPosition.NET_PREP, ElevatorLevel.NET_PREP);
      
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
    public static final double ALGAE_CURRENT_THRESHOLD = 60.0; // amps
    public static final double ZERO_DEBOUNCE_TIME = 0.2;
    public static final double ZEROING_SPEED = -0.065; // Slow inwards speed
    public static final double ALGAE_DETECTION_DEBOUNCE_TIME = 0.3; // 100ms debounce time

    // Intake constantsd
    public static final double CORAL_INTAKE_SPEED = -5; // Rps
    public static final double ALGAE_HOLD_SPEED = 30; // Speed to hold algae in place
    public static final double ALGAE_INTAKE_SPEED = 120;
    public static final double FAST_CORAL_INTAKE_SPEED = -10;

    // Pivot constants
    public static final double PIVOT_ANGLE_DEADBAND = 1.4;
    public static final double PIVOT_MIN_ANGLE = 0.0; // degrees
    public static final double PIVOT_MAX_ANGLE = 185.0; // degrees
    public static final double PIVOT_BUMPER_CLEARANCE_ANGLE = 150; // degrees
    public static final double PIVOT_L4_CLEARANCE_ANGLE = 28;

    // Motor configuration
    public static final double PIVOT_MOTION_CRUISE_VELOCITY = 6;
    public static final double PIVOT_MOTION_ACCELERATION = 30.0;
    public static final double PIVOT_MOTION_JERK = 2000.0;
    public static final double STATOR_CURRENT_LIMIT = 50.0; // amps
    public static final double PIVOT_MOTOR_DEADBAND = 0.001;
    public static final double PIVOT_SUPPLY_VOLTAGE = 12.0;
    public static final double PIVOT_CURRENT_THRESHOLD =  27.0; // amps - Current threshold for zeroing

    // PID Values
    public static final double PIVOT_kP = 80.0;
    public static final double PIVOT_kI = 0.0;
    public static final double PIVOT_kD = 0.0;
    public static final double PIVOT_kS = 0.15;

    public static final double PIVOT_kG_HORIZONTAL = -0.3; 

    public static final double PIVOT_kP_ALGAE_SLOW = 40.0;

    // Pivot Positions
    public enum PivotPosition {
        ZERO(0),
        CLEARANCE_POSITION(35),
        CLEARANCE_POSITION_ALGAE(106),
        ALGAE_L2(178.5),
        ALGAE_L1(178.5),
        PROCESSOR(189),
        SPIT_ALGAE(140),
        CORAL_INTAKE(2.6),
        NET(98),
        L4(66),
        L3(34),
        L2(34),
        L1(150),

        // Maybe manual mode
        MANUAL_L4(71.0),
        MANUAL_L3(24.0),
        MANUAL_L2(24.0),
        MANUAL_L1(150),

        NET_PREP(180);

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
    public static final double STALL_CURRENT_THRESHOLD = 26.0; // Amperes
    public static final double ZERO_DEBOUNCE_TIME = 0.2;

    // Elevator will not move if the pivot is not past this angle, to avoid collision with top bar
    public static final double MIN_ELEVATOR_PIVOT_ANGLE = 32; 
    public static final double PIVOT_BUMPER_CLEAR_HEIGHT = 0.13;

    public static final double PIVOT_L4_CLEAR_HEIGHT_MIN = 0.70;
    public static final double PIVOT_L4_CLEAR_HEIGHT_MAX = 1.2;

    public static final double MIN_ELEVATOR_HEIGHT = 0;
    public static final double MAX_ELEVATOR_HEIGHT = 1.50;

    // Collision zone constants
    public static final double COLLISION_ZONE_LOWER = 0.16; // meters
    public static final double COLLISION_ZONE_UPPER = 0.61; // meters

    // Height where first stage starts moving
    public static final double FIRST_STAGE_START_HEIGHT = ElevatorConstants.MAX_ELEVATOR_HEIGHT / 2.0; 

    // Motion Magic configuration
    public static final double MOTION_CRUISE_VELOCITY = 6; // 4 usually
    public static final double MOTION_ACCELERATION = 4;
    public static final double MOTION_JERK = 2000;

    // PID Values
    public static final double kP = 60.0;
    public static final double kI = 0.0;
    public static final double kD = 0.01;
    public static final double kS = 0.0;
    public static final double kG = 0.36; 

    // Predefined heights for the elevator (in meters)
    public enum ElevatorLevel {
      REST_POSITION(0.0),
      NET(1.50),
      ALGAE_L2(0.88),
      ALGAE_L1(0.54),
      PROCESSOR(0.135),
      L4(1.50),
      L3(0.865),
      L2(0.44),
      L1(0.33),

      MANUAL_L4(1.440),
      MANUAL_L3(0.6772),
      MANUAL_L2(0.280),
      MANUAL_L1(0.33),

      NET_PREP(1.3);


      private final double height;

      ElevatorLevel(double height) {
        this.height = height;
      }

      public double getHeight() {
        return height;
      }
    }
  }
  
  /* Shark Pivot Constants */
  public static class SharkPivotConstants {
    // Control constants
    public static final double DEAD_ZONE = 5.0; // In degrees
    public static final double MIN_ANGLE = 0.0; // Minimum angle in degrees
    public static final double MAX_ANGLE = 195.0; // Maximum angle in degrees - adjust as needed

    // Zeroing
    public static final double ZEROING_SPEED = -0.05;
    public static final double ZERO_DEBOUNCE_TIME = 0.2; // seconds
    public static final double PIVOT_CURRENT_THRESHOLD = 20; // amps

    // Motor configuration
    public static final double STATOR_CURRENT_LIMIT = 40.0; // amps
    public static final double MOTION_CRUISE_VELOCITY = 4; 
    public static final double MOTION_ACCELERATION = 12; 
    public static final double MOTION_JERK = 2000.0; 

    // PID Values
    public static final double kP = 70.0;
    public static final double kI = 0.0;
    public static final double kD = 0.01;
    public static final double kS = 0.0;

    // Predefined positions for the shark (in degrees)
    public enum SharkPivotPosition {
      STOWED(0.0),
      DEPLOYED(138.0),
      L1(27);

      private final double degrees;

      SharkPivotPosition(double degrees) {
        this.degrees = degrees;
      }

      public double getDegrees() {
        return degrees;
      }
    }
  }

  /* Shark Intake Constants */
  public static class SharkIntakeConstants {
    // PID Values
    public static final double kP = 60.0;
    public static final double kI = 0.0;
    public static final double kD = 0.01;
    public static final double kS = 0.0;

    public static final double CORAL_CURRENT_THRESHOLD = 24.0; // amps
    public static final double CORAL_EJECT_VELOCITY_THRESHOLD = -1.6; // rps

    public static final double STATOR_CURRENT_LIMIT = 60;
  }
}
