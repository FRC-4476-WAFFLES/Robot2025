// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controls;
import frc.robot.RobotContainer;
import frc.robot.data.Constants;
import frc.robot.subsystems.DynamicPathing.DynamicPathingSituation;
import frc.robot.subsystems.GroundSuperstructure.GroundIntakeSuperstructure.GroundIntakeSuperstructureState;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;

public class Lights extends SubsystemBase {
  private static final int LED_COUNT = 186;
  private static final double DEFAULT_BLINK_RATE = 0.1;
  private static final int FLOW_LENGTH = 32;
  private static final double LED_UPDATE_RATE = 0.05; 
  
  private static final CANdle candle = new CANdle(Constants.CANIds.CANdle);
  
  private Map<LedRange, LightColours> ledRangeColours = new EnumMap<>(LedRange.class);
  private Map<LedRange, LightColours> lastAppliedColors = new EnumMap<>(LedRange.class);
  
  private static final Timer blinkTimer = new Timer();
  private static final Timer animationTimer = new Timer();
  private boolean isBlinkColour = true;
  private double blinkRate = DEFAULT_BLINK_RATE;
  
  private int flowPosition = 8;
  private boolean isCoralIntakeRunning = false;
  
  private int rainbowOffset = 0;
  private boolean rainbowActive = false;
  private Set<LedRange> rainbowRanges = new HashSet<>();
  
  private enum UpdateMode { STATIC, ANIMATED }
  private Map<LedRange, UpdateMode> rangeUpdateModes = new EnumMap<>(LedRange.class);

  /**
   * Enum containing start and and indicies for various defined LED groups
   */
  public enum LedRange {
    CANDLE(0,8),
    // Full sections
    RIGHT_SIDE_FULL(129,186),//59
    MIDDLE_FULL(68,128),//60
    LEFT_SIDE_FULL(8,67), //59

    // Right side
    R1(173,186),      // Bottom section
    R2(158,186),      // Bottom + lower middle
    R3(144,186),      // Bottom + lower middle + upper middle

    // Middle sections
    MIDDLE_LEFT(68,88),      // First third of middle
    MIDDLE_MIDDLE(88,108),   // Second third of middle
    MIDDLE_RIGHT(108,127),   // Final third of middle
    
    // Left side sections and progressive ranges
    L1(8,23),         // Bottom section
    L2(8,38),         // Bottom + lower middle
    L3(8,52);         // Bottom + lower middle + upper middle

  

    private final int start;
    private final int end;

    LedRange(int start, int end) {
      this.start = start;
      this.end = end;
    }

    public int getStart() {
      return start;
    }

    public int getEnd() {
      return end;
    }
  }

  /**
   * Enum containing commonly used RGB colors
   */
  public enum LightColours {
    BLACK(0, 0, 0),
    BROWN(96, 32, 8),
    INFRARED(50, 0, 0),
    RED(255, 0, 0),
    LIGHTRED(255, 105, 105),
    SUN(255, 60, 0),
    ORANGE(255, 18, 0),
    YELLOW(255, 190, 0),
    LIME(187, 255, 0),
    LIGHTGREEN(130, 247, 119),
    GREEN(0, 255, 0),
    DARKGREEN(21, 102, 13),
    CYAN(0, 255, 179),
    LIGHTBLUE(103, 120, 214),
    BLUE(0, 0, 255),
    NAVY(9, 15, 79),
    ULTRAVIOLET(50, 0, 100),
    PURPLE(150, 0, 255),
    MAGENTA(150, 15, 92),
    PINK(255, 0, 255),
    WHITE(255, 255, 255),
    GRAY(127, 127, 127),

    FLOW_COLOR(255, 190, 0);

    private final int red;
    private final int green;
    private final int blue;

    private int[] packedColors;

    LightColours(int red, int green, int blue) {
      this.red = red;
      this.green = green;
      this.blue = blue;

      // cache array to reduce allocations
      this.packedColors = new int[]{red, green, blue};
    }

    public int[] getRGBValues() {
      return packedColors;
    }
  }

  /**
   * CANDLE Hardware Animations
   */
  public enum LedAnimation {
    STROBE(new StrobeAnimation(255, 0, 0, 0, 98.0 / 256.0, LED_COUNT)),
    LARSON(new LarsonAnimation(255, 255, 0, 0, 0.2, LED_COUNT, BounceMode.Front, 2)),
    COLOR_FLOW(new ColorFlowAnimation(255, 255, 0, 0, 0.05, LED_COUNT, Direction.Forward)),
    RAINBOW(new RainbowAnimation(0.9, 0.1, LED_COUNT));

    private final Animation animation;

    LedAnimation(Animation animation) {
        this.animation = animation;
    }

    public Animation getAnimation() {
        return animation;
    }
  }

  /**
   * Constructs a new LightSubsystem.
   * Initializes the blink timer and configures the CANdle settings.
   */
  public Lights() {
    blinkTimer.reset();
    blinkTimer.start();

    CANdleConfiguration configAll = new CANdleConfiguration();

    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 0.75;
    configAll.vBatOutputMode = VBatOutputMode.On;
    configAll.v5Enabled = true;

    candle.configAllSettings(configAll, 1000);

    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) {
      candle.clearAnimation(i);
    }
    candle.animate(null);
    
    animationTimer.reset();
    animationTimer.start();
    
    for (LedRange range : LedRange.values()) {
      rangeUpdateModes.put(range, UpdateMode.STATIC);
    }
  }


  @Override
  public void periodic() {
    if (RobotBase.isSimulation()) {
      return;
    }

    ledRangeColours.clear();

    if (DriverStation.isEnabled()) {
      handleEnabledState();
    } else {
      handleDisabledState();
    }

    updateBlinkTimer();
    applyLEDRanges();
  }

  private void handleEnabledState() {
    if (!RobotContainer.isOperatorOverride) {
      handleAutomaticElevatorLights();
      clearAllRainbow();
    } else {
      handleManualElevatorLights();
      enableManualModeRainbow();
    }
    
    updatePathingIndicators();
    updateOverrideIndicators();
    
    if (rainbowActive && animationTimer.get() > LED_UPDATE_RATE) {
      updateRainbow();
      animationTimer.reset();
    }
  }

  /**
   * Updates the following operator override indicators
   *  - Do not score 
   *  - Is in override mode
   */ 
  private void updateOverrideIndicators() {
    if (RobotContainer.isOperatorOverride) {
      // Update green central range if in override mode
      setLEDRangeGroup(LedRange.MIDDLE_MIDDLE, LightColours.GREEN, LightColours.BLACK, false);

    } else if (Controls.doNotScore.getAsBoolean()) {
      // When "do not score" is active, and we are in normal mode, set middle lights to red
      setLEDRangeGroup(LedRange.MIDDLE_MIDDLE, LightColours.RED, LightColours.BLACK, false);

    } else {
      // Clear central LEDs if no state is being displayed
      setLEDRangeGroup(LedRange.MIDDLE_MIDDLE, LightColours.BLACK, LightColours.BLACK, false);
    }
  }

  private void handleDisabledState() {
    clearHardwareAnimations();

    updateDiagnosticIndicators();
    
    if (animationTimer.get() > LED_UPDATE_RATE) {
      if (rainbowActive) {
        updateRainbow();
      } else {
        updateFlowAnimation();
      }
      animationTimer.reset();
    }
  }
  
  private void updateFlowAnimation() {
    int clearStart = flowPosition - FLOW_LENGTH;
    if (clearStart < 8) {
      clearStart = LED_COUNT - (8 - clearStart);
    }
    
    flowPosition = (flowPosition + 2) % LED_COUNT;
    if (flowPosition < 8) {
      flowPosition = 8;
    }
    
    int segmentLength = Math.min(FLOW_LENGTH, LED_COUNT - flowPosition);
    if (segmentLength > 0) {
      candle.setLEDs(LightColours.FLOW_COLOR.red, 
                     LightColours.FLOW_COLOR.green, 
                     LightColours.FLOW_COLOR.blue, 
                     0, flowPosition, segmentLength);
    }
    
    if (clearStart >= 8 && clearStart < LED_COUNT) {
      int clearLength = Math.min(FLOW_LENGTH, LED_COUNT - clearStart);
      candle.setLEDs(0, 0, 0, 0, clearStart, clearLength);
    }
  }


  /**
   * Indicators used to perform systems check
   */
  private void updateDiagnosticIndicators() {
    // Algae Loaded
    if (RobotContainer.intakeSubsystem.isAlgaeLoaded()) {
      setLEDRange(0, 1, LightColours.DARKGREEN);
    } else {
      setLEDRange(0, 1, LightColours.BLACK);
    }
    
    // Coral Loaded
    if (RobotContainer.intakeSubsystem.isCoralLoaded()) {
      setLEDRange(1, 2, LightColours.WHITE);
    } else {
      setLEDRange(1, 2, LightColours.BLACK);
    }

    // Add pivot position indicator
    double pivotPosition = RobotContainer.superstructure.pivot.getPivotPosition();
    if (Math.abs(pivotPosition) <= 2.0) { // Within 2 degrees of zero
      setLEDRange(2, 3, LightColours.BLUE);
    } else {
      setLEDRange(2, 3, LightColours.BLACK);
    }

    // Add elevator position indicator 
    double elevatorPosition = RobotContainer.superstructure.elevator.getElevatorPositionMeters();
    if (Math.abs(elevatorPosition) <= 0.02) { // Within 2cm of zero
      setLEDRange(3, 4, LightColours.CYAN);
    } else {
      setLEDRange(3, 4, LightColours.BLACK);
    }

    // Can see tag indicator
    if (RobotContainer.driveSubsystem.limelightsSeeTag()) {
      setLEDRange(4, 5, LightColours.PINK);
    } else {
      setLEDRange(4, 5, LightColours.BLACK);
    }

    // Alliance Indicator Light
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Blue) {
        setLEDRange(6, 8, LightColours.BLUE);
      } else {
        setLEDRange(6, 8, LightColours.RED);
      }
    } else {
      setLEDRange(6, 8, LightColours.BLUE);
    }
  }

  /**
   * Updates elevator side lights based on targeted elevator height in automatic mode
   */
  private void handleAutomaticElevatorLights() {
    if (RobotContainer.isHeadingLockedToL1.getAsBoolean()) {
      setLEDRangeGroup(LedRange.LEFT_SIDE_FULL, LightColours.ORANGE, LightColours.BLACK, true);
      setLEDRangeGroup(LedRange.RIGHT_SIDE_FULL, LightColours.ORANGE, LightColours.BLACK, true);
      return; 
    }
    if (RobotContainer.groundSuperstructure.isL1Ready()) {
      setLEDRangeGroup(LedRange.LEFT_SIDE_FULL, LightColours.GREEN, LightColours.BLACK, false);
      setLEDRangeGroup(LedRange.RIGHT_SIDE_FULL, LightColours.GREEN, LightColours.BLACK, false);
      return;
    } 
    if (RobotContainer.groundSuperstructure.getState() == GroundIntakeSuperstructureState.INTAKE_L1_STATE) {
      // Flash green if intaking
      setLEDRangeGroup(LedRange.LEFT_SIDE_FULL, LightColours.GREEN, LightColours.BLACK, true);
      setLEDRangeGroup(LedRange.RIGHT_SIDE_FULL, LightColours.GREEN, LightColours.BLACK, true);
      return;
    }

    SuperstructureState scoringLevel = RobotContainer.dynamicPathingSubsystem.getCoralScoringLevel();
    boolean isRightSide = RobotContainer.dynamicPathingSubsystem.getCoralScoringSide();
    
    switch (scoringLevel) {
      case L1:
        setElevatorRange(LedRange.L1, isRightSide);
        setElevatorRange(LedRange.R1, isRightSide);
        break;
      case L2:
        setElevatorRange(LedRange.L2, isRightSide);
        setElevatorRange(LedRange.R2, isRightSide);
        break;
      case L3:
        setElevatorRange(LedRange.L3, isRightSide);
        setElevatorRange(LedRange.R3, isRightSide);
        break;
      case L4:
        setElevatorRange(LedRange.LEFT_SIDE_FULL, isRightSide);
        setElevatorRange(LedRange.RIGHT_SIDE_FULL, isRightSide);
        break;
      default:
        break;
    }
  }

  /**
   * Helper methods for elevator LEDs 
   */
  private void setElevatorRange(LedRange range, boolean isRightSide) {
    LightColours color = isRightSide ? LightColours.PURPLE : LightColours.YELLOW;
    setLEDRangeGroup(range, color, LightColours.WHITE, false);
  }

  /**
   * Updates elevator side lights based on current elevator height setpoint in manual mode
   */
  private void handleManualElevatorLights() {
    SuperstructureState elevatorLevel =  RobotContainer.superstructure.elevator.getElevatorSetpointEnum();
    boolean hasCoralLoaded = RobotContainer.intakeSubsystem.isCoralLoaded();
    setElevatorLevelPattern(elevatorLevel, hasCoralLoaded);
  }

  /*
   * Helper methods for elevator LEDs 
   */
  private void setElevatorLevelPattern(SuperstructureState level, boolean isCoralLoaded) {
    LedRange leftRange = null;
    LedRange rightRange = null;
    LightColours color = isCoralLoaded ? LightColours.WHITE : LightColours.BLACK;

    switch (level) {
      case L1:
        leftRange = LedRange.L1;
        rightRange = LedRange.R1;
        break;
      case PROCESSOR:
        leftRange = LedRange.L1;
        rightRange = LedRange.R1;
      case L2:
        leftRange = LedRange.L2;
        rightRange = LedRange.R2;
        break;
      case ALGAE_L1:
        leftRange = LedRange.L2;
        rightRange = LedRange.R2;
        break;
      case L3:
        leftRange = LedRange.L3;
        rightRange = LedRange.R3;
        break;
      case ALGAE_L2:
        leftRange = LedRange.L3;
        rightRange = LedRange.R3;
        break;
      case L4:
        leftRange = LedRange.LEFT_SIDE_FULL;
        rightRange = LedRange.RIGHT_SIDE_FULL;
        break;
      case NET:
        leftRange = LedRange.LEFT_SIDE_FULL;
        rightRange = LedRange.RIGHT_SIDE_FULL;
        break;
      default:
        setLEDRangeGroup(LedRange.LEFT_SIDE_FULL, LightColours.BLACK, LightColours.BLACK, false);
        setLEDRangeGroup(LedRange.RIGHT_SIDE_FULL, LightColours.BLACK, LightColours.BLACK, false);
        return;
    }

    // Finally apply chosen range and color
    setLEDRangeGroup(leftRange, color, LightColours.WHITE, false);
    setLEDRangeGroup(rightRange, color, LightColours.WHITE, false);
  }

  /**
   * Update dynamic pathing indicators
   */
  private void updatePathingIndicators() {
    var pathingSituation = RobotContainer.dynamicPathingSubsystem.getCurrentPathingSituation();

    if (isCoralIntakeRunning) {
      // When coral intake is running, blink white lights
      setLEDRangeGroup(LedRange.MIDDLE_LEFT, LightColours.WHITE, LightColours.BLACK, true);
      setLEDRangeGroup(LedRange.MIDDLE_RIGHT, LightColours.WHITE, LightColours.BLACK, true);

    } else if (pathingSituation == DynamicPathingSituation.REEF_CORAL) {
      setLEDRangeGroup(LedRange.MIDDLE_LEFT, LightColours.WHITE, LightColours.BLACK, false);
      setLEDRangeGroup(LedRange.MIDDLE_RIGHT, LightColours.WHITE, LightColours.BLACK, false);

    } else if (pathingSituation == DynamicPathingSituation.REEF_ALGAE) {
      setLEDRangeGroup(LedRange.MIDDLE_LEFT, LightColours.DARKGREEN, LightColours.BLACK, false);
      setLEDRangeGroup(LedRange.MIDDLE_RIGHT, LightColours.DARKGREEN, LightColours.BLACK, false);
    
    } else if (pathingSituation == DynamicPathingSituation.HUMAN_PICKUP) {
      setLEDRangeGroup(LedRange.MIDDLE_LEFT, LightColours.RED, LightColours.BLACK, false);
      setLEDRangeGroup(LedRange.MIDDLE_RIGHT, LightColours.RED, LightColours.BLACK, false);
    
    } else if (pathingSituation == DynamicPathingSituation.PROCESSOR) {
      setLEDRangeGroup(LedRange.MIDDLE_LEFT, LightColours.BLUE, LightColours.BLACK, false);
      setLEDRangeGroup(LedRange.MIDDLE_RIGHT, LightColours.BLUE, LightColours.BLACK, false);

    } else if (pathingSituation == DynamicPathingSituation.NET) {
      setLEDRangeGroup(LedRange.MIDDLE_LEFT, LightColours.PINK, LightColours.BLACK, false);
      setLEDRangeGroup(LedRange.MIDDLE_RIGHT, LightColours.PINK, LightColours.BLACK, false);

    } else {
      setLEDRangeGroup(LedRange.MIDDLE_LEFT, LightColours.BLACK, LightColours.BLACK, false);
      setLEDRangeGroup(LedRange.MIDDLE_RIGHT, LightColours.BLACK, LightColours.BLACK, false);
    }
  }

  /*                       */
  /* Light Utility Methods */
  /*                       */

  /**
   * Updates the blink timer and toggles the blink state if necessary.
   */
  private void updateBlinkTimer() {
    if (blinkTimer.get() > blinkRate) {
        isBlinkColour = !isBlinkColour;
        blinkTimer.reset();
    }
  }

  /**
   * Sets the blink rate for LED animations.
   * @param seconds The time in seconds between blinks
   */
  public void setBlinkTime(double seconds) {
    blinkRate = seconds;
  }

  /**
   * Sets the LED color for a specific range of LEDs.
   * @param start The starting index of the LED range
   * @param end The ending index of the LED range
   * @param colour The color to set for the LED range
   */
  public void setLEDRange(int start, int end, LightColours colour) {
    candle.setLEDs(colour.red, colour.green, colour.blue, 0, start, end-start);
  }

  /**
   * Sets the LED color for a predefined LED range group, with optional blinking.
   * @param range The predefined LED range
   * @param colour The primary color for the LED range
   * @param blinkColour The secondary color for blinking (if enabled)
   * @param shouldBlink Whether the LED range should blink
   */
  public void setLEDRangeGroup(LedRange range, LightColours colour, LightColours blinkColour, boolean shouldBlink) {
    rangeUpdateModes.put(range, shouldBlink ? UpdateMode.ANIMATED : UpdateMode.STATIC);
    
    if(shouldBlink){
      if(isBlinkColour) {
        ledRangeColours.put(range, colour);
      } else {
        ledRangeColours.put(range, blinkColour);
      } 
    } else {
      ledRangeColours.put(range, colour);
    }
  }

  /**
   * Sets all LEDs to a single color.
   * @param colour The color to set for all LEDs
   */
  public void setAllLEDs(LightColours colour) {
    ledRangeColours.clear();
    for (LedRange range : LedRange.values()) {
      ledRangeColours.put(range, colour);
    }
  }

  private void applyLEDRanges() {
    if (ledRangeColours.isEmpty()) {
      return;
    }

    for (Map.Entry<LedRange, LightColours> entry : ledRangeColours.entrySet()) {
      LedRange range = entry.getKey();
      LightColours newColour = entry.getValue();
      
      LightColours lastColour = lastAppliedColors.get(range);
      UpdateMode updateMode = rangeUpdateModes.getOrDefault(range, UpdateMode.STATIC);
      
      if (updateMode == UpdateMode.STATIC && newColour.equals(lastColour)) {
        continue;
      }
      
      candle.setLEDs(newColour.red, newColour.green, newColour.blue, 0, 
                     range.getStart(), range.getEnd() - range.getStart());
      
      lastAppliedColors.put(range, newColour);
    }
  }

  public void clearAllLEDs() {
    candle.setLEDs(0, 0, 0, 0, 0, LED_COUNT);
    ledRangeColours.clear();
    lastAppliedColors.clear();
    clearAllRainbow();
  }

  /**
   * Clears all CANDLE hardware animations
   */
  public void clearHardwareAnimations() {
    candle.animate(null);
    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) {
      candle.clearAnimation(i);
    }
  }

  /**
   * Sets the light's is intaking state
   * @param running is intaking
   */
  public void setCoralIntakeRunning(boolean running) {
    isCoralIntakeRunning = running;
  }
  
  public void setRainbowMode(LedRange range, boolean enabled) {
    if (enabled && range != null) {
      rainbowRanges.add(range);
      rangeUpdateModes.put(range, UpdateMode.ANIMATED);
      rainbowActive = true;
    } else if (range != null) {
      rainbowRanges.remove(range);
      rangeUpdateModes.put(range, UpdateMode.STATIC);
    }
    
    if (rainbowRanges.isEmpty()) {
      rainbowActive = false;
    }
  }
  
  public void celebrationMode() {
    setRainbowMode(LedRange.MIDDLE_FULL, true);
  }
  
  public void clearCelebrationMode() {
    setRainbowMode(LedRange.MIDDLE_FULL, false);
  }
  
  public void clearAllRainbow() {
    for (LedRange range : new HashSet<>(rainbowRanges)) {
      setRainbowMode(range, false);
    }
  }
  
  private void enableManualModeRainbow() {
    SuperstructureState elevatorLevel = RobotContainer.superstructure.elevator.getElevatorSetpointEnum();
    boolean hasCoralLoaded = RobotContainer.intakeSubsystem.isCoralLoaded();
    
    if (!hasCoralLoaded) {
      switch (elevatorLevel) {
        case L1:
        case PROCESSOR:
          setRainbowMode(LedRange.L1, true);
          setRainbowMode(LedRange.R1, true);
          break;
        case L2:
        case ALGAE_L1:
          setRainbowMode(LedRange.L2, true);
          setRainbowMode(LedRange.R2, true);
          break;
        case L3:
        case ALGAE_L2:
          setRainbowMode(LedRange.L3, true);
          setRainbowMode(LedRange.R3, true);
          break;
        case L4:
        case NET:
          setRainbowMode(LedRange.LEFT_SIDE_FULL, true);
          setRainbowMode(LedRange.RIGHT_SIDE_FULL, true);
          break;
        default:
          setRainbowMode(null, false);
          break;
      }
    }
  }
  
  private void updateRainbow() {
    if (!rainbowActive || rainbowRanges.isEmpty()) return;
    
    rainbowOffset = (rainbowOffset + 15) % 255;
    
    int r = (int)(127 + 127 * Math.sin(rainbowOffset * 0.024));
    int g = (int)(127 + 127 * Math.sin(rainbowOffset * 0.024 + 2.094));
    int b = (int)(127 + 127 * Math.sin(rainbowOffset * 0.024 + 4.188));
    
    for (LedRange range : rainbowRanges) {
      candle.setLEDs(r, g, b, 0, range.getStart(), 
                    range.getEnd() - range.getStart());
    }
  }
} 