// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import static frc.robot.RobotContainer.dynamicPathingSubsystem;
import static frc.robot.RobotContainer.elevatorSubsystem;
import static frc.robot.RobotContainer.intakeSubsystem;
import frc.robot.data.Constants;
import frc.robot.data.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.data.Constants.ScoringConstants.ScoringLevel;
import frc.robot.subsystems.Lights.LightColours;
import frc.robot.utils.LimelightHelpers;

public class Lights extends SubsystemBase {
  // Constants
  private static final int LED_COUNT = 122;
  private static final double DEFAULT_BLINK_RATE = 0.1;
  
  // Hardware
  private static final CANdle candle = new CANdle(Constants.CANIds.CANdle);
  
  // State
  private static final Timer blinkTimer = new Timer();
  private boolean isBlinkColour = true;
  public boolean isEndgameWarning = false;
  private double blinkRate = DEFAULT_BLINK_RATE;
  private int[][] ledColors;
  private Map<LedRange, LightColours> ledRangeColours = new EnumMap<>(LedRange.class);
  private int flowPosition = 8; // Start flow at LED 8
  private static final int FLOW_LENGTH = 20; // Length of the flowing section

  public enum LedRange {
    CANDLE(0,8),
    // Full sections
    RIGHT_SIDE_FULL(84,122),//37
    MIDDLE_FULL(46,83),//36
    LEFT_SIDE_FULL(8,45), //37
    // Right side sections and progressive ranges
    RIGHT_SIDE_TOP(8,15),
    RIGHT_SIDE_UPPER_MIDDLE(15,23),
    RIGHT_SIDE_LOWER_MIDDLE(23,31),
    RIGHT_SIDE_BOTTOM(31,38),
    R1(113,122),      // Bottom only
    R2(104,122),      // Bottom + lower middle
    R3(95,122),      // Bottom + lower middle + upper middle

    // Middle sections
    MIDDLE_LEFT(46,58),
    MIDDLE_MIDDLE(59,70),
    MIDDLE_RIGHT(71,83),
    // Left side sections and progressive ranges
    LEFT_SIDE_TOP(62,71),
    LEFT_SIDE_UPPER_MIDDLE(71,80),
    LEFT_SIDE_LOWER_MIDDLE(80,89),
    LEFT_SIDE_BOTTOM(89,100),
    L1(8,17),     // Bottom only
    L2(8,26),     // Bottom + lower middle
    L3(8,35);     // Bottom + lower middle + upper middle
  

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

  public enum LightColours {
    BLACK(0, 0, 0),
    BROWN(96, 32, 8),
    INFRARED(50, 0, 0),
    RED(255, 0, 0),
    LIGHTRED(255, 105, 105),
    SUN(255, 60, 0),
    ORANGE(255, 18, 0),
    YELLOW(255, 255, 0),
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
    GRAY(127, 127, 127);

    private final int red;
    private final int green;
    private final int blue;

    LightColours(int red, int green, int blue) {
      this.red = red;
      this.green = green;
      this.blue = blue;
    }

    public int[] getRGBValues() {
      return new int[]{red, green, blue};
    }
  }

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

    configAll.stripType = LEDStripType.RGB;
    configAll.brightnessScalar = 1;
    configAll.vBatOutputMode = VBatOutputMode.On;

    candle.configAllSettings(configAll, 1000);
    candle.configLEDType(LEDStripType.RGB);

    // Explicitly disable all animations
    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) {
      candle.clearAnimation(i);
    }
    candle.animate(null);

    ledColors = new int[LED_COUNT][3];
  }

  /**
   * This method is called periodically by the CommandScheduler.
   * It updates the LED animations based on the robot's state.
   */
  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      handleDisabledState();
    } else {
      handleEnabledState();
    }
  }

  private void handleDisabledState() {
    // First update status indicators
    updateIntakeIndicators();
    updateAprilTagIndicator();
    updateBlinkTimer();
    updateLedRanges();
    
    // Make sure no animations are running
    candle.animate(null);
    
    // Update flow position
    if (blinkTimer.get() > 0.05) { // Control flow speed
      // Calculate and clear previous tail LED
      int tailPos = flowPosition - FLOW_LENGTH;
      if (tailPos < 8) {
        tailPos = LED_COUNT - (8 - tailPos);
      }
      if (tailPos >= 8 && tailPos < LED_COUNT) {
        candle.setLEDs(0, 0, 0, 0, tailPos, 1);
      }
      
      // Update position
      flowPosition = (flowPosition + 1) % LED_COUNT;
      if (flowPosition < 8) {
        flowPosition = 8;
      }
      
      // Set new head LED to full brightness
      candle.setLEDs(255, 255, 0, 0, flowPosition, 1);
      
      blinkTimer.reset();
    }
  }

  private void handleEnabledState() {
    // Make sure no animations are running
    candle.animate(null);
    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) {
      candle.clearAnimation(i);
    }
    
    updateLights();  // Update standard LED patterns
    updateBlinkTimer();
    updateLedRanges();
  }

  private void updateIntakeIndicators() {
    if (intakeSubsystem.isAlgaeLoaded()) {
      setLEDRange(0, 1, LightColours.DARKGREEN);
    } else {
      setLEDRange(0, 1, LightColours.BLACK);
    }
    
    if (intakeSubsystem.isCoralLoaded()) {
      setLEDRange(1, 2, LightColours.WHITE);
    } else {
      setLEDRange(1, 2, LightColours.BLACK);
    }
  }

  private void updateAprilTagIndicator() {
    if (LimelightHelpers.getTV("limelight-front")) {
      setLEDRange(1, 2, LightColours.GREEN);
    } else {
      setLEDRange(1, 2, LightColours.BLACK);
    }
  }

  /**
   * Determines the appropriate LED animation based on the robot's state.
   * @return The LedAnimation to be displayed
   */
  private LedAnimation getLedAnimation() {
    return LedAnimation.COLOR_FLOW;
  }

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
   * @param canBlink Whether the LED range should blink
   */
  public void setLEDRangeGroup(LedRange range, LightColours colour, LightColours blinkColour, boolean canBlink) {
    if(canBlink){
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

   private void updateLedRanges() {
    if (ledRangeColours.isEmpty()) {
      return; // Don't update if there are no colors to set
    }

    // Initialize ledColors[] to default color
    int[] defaultRGB = LightColours.BLACK.getRGBValues();
    for (int i = 0; i < LED_COUNT; i++) {
        ledColors[i][0] = defaultRGB[0];
        ledColors[i][1] = defaultRGB[1];
        ledColors[i][2] = defaultRGB[2];
    }

    // Convert ledRangeColours entries to a list for sorting
    List<Map.Entry<LedRange, LightColours>> entries = new ArrayList<>(ledRangeColours.entrySet());

    // Sort ranges from largest to smallest to give precedence to smaller ranges
    entries.sort((entry1, entry2) -> {
        int size1 = entry1.getKey().getEnd() - entry1.getKey().getStart();
        int size2 = entry2.getKey().getEnd() - entry2.getKey().getStart();
        return Integer.compare(size2, size1); // Largest size first
    });

    // Apply colors to ledColors[] array
    for (Map.Entry<LedRange, LightColours> entry : entries) {
        LedRange range = entry.getKey();
        LightColours colour = entry.getValue();
        int[] rgb = colour.getRGBValues();
        for (int i = range.getStart(); i < range.getEnd(); i++) {
            ledColors[i][0] = rgb[0];
            ledColors[i][1] = rgb[1];
            ledColors[i][2] = rgb[2];
        }
    }

    // Optimize LED updates by grouping contiguous colors
    int idx = 0;
    while (idx < LED_COUNT) {
        int[] currentColor = ledColors[idx];
        int startIdx = idx;
        int count = 1;
        idx++;
        while (idx < LED_COUNT && Arrays.equals(ledColors[idx], currentColor)) {
            count++;
            idx++;
        }
        // Update the LEDs for this contiguous range
        candle.setLEDs(currentColor[0], currentColor[1], currentColor[2], 0, startIdx, count);
    }
  }


  /**
   * Clears all LEDs by setting them to black.
   */
  public void clearAllLEDs() {
    for (int i = 0; i < LED_COUNT; i++) {
      ledColors[i] = LightColours.BLACK.getRGBValues();
    }
    ledRangeColours.clear();
    updateLedRanges();
  }

  /**
   * Updates all lights based on current robot state.
   * This method is called by the default command to handle standard lighting patterns.
   */
  public void updateLights() {
    if (elevatorSubsystem.getElevatorSetpointEnum() == ElevatorLevel.REST_POSITION) {
      handleRestPositionLights();
    } else {
      handleElevatorPositionLights();
    }
    
    updateReefPathingIndicators();
    updateOperatorOverrideIndicator();
    updateLedRanges();
  }

  private void handleRestPositionLights() {
    ScoringLevel scoringLevel = dynamicPathingSubsystem.getCoralScoringLevel();
    boolean isRightSide = dynamicPathingSubsystem.getCoralScoringSide();
    
    switch (scoringLevel) {
      case L1:
        setCoralScoringPattern(LedRange.L1, isRightSide);
        setCoralScoringPattern(LedRange.R1, isRightSide);
        break;
      case L2:
        setCoralScoringPattern(LedRange.L2, isRightSide);
        setCoralScoringPattern(LedRange.R2, isRightSide);
        break;
      case L3:
        setCoralScoringPattern(LedRange.L3, isRightSide);
        setCoralScoringPattern(LedRange.R3, isRightSide);
        break;
      case L4:
        setCoralScoringPattern(LedRange.LEFT_SIDE_FULL, isRightSide);
        setCoralScoringPattern(LedRange.RIGHT_SIDE_FULL, isRightSide);
        break;
    }
  }

  private void handleElevatorPositionLights() {
    ElevatorLevel elevatorLevel = elevatorSubsystem.getElevatorSetpointEnum();
    boolean hasCoralLoaded = intakeSubsystem.isCoralLoaded();
    setElevatorLevelPattern(elevatorLevel, hasCoralLoaded);
  }

  private void updateReefPathingIndicators() {
    if (DynamicPathingSubsystem.isRobotInRangeOfReefPathing()) {
      LightColours color = intakeSubsystem.isCoralLoaded() ? LightColours.WHITE : LightColours.DARKGREEN;
      setLEDRangeGroup(LedRange.MIDDLE_LEFT, color, LightColours.BLACK, false);
      setLEDRangeGroup(LedRange.MIDDLE_RIGHT, color, LightColours.BLACK, false);
    } else {
      setLEDRangeGroup(LedRange.MIDDLE_LEFT, LightColours.BLACK, LightColours.BLACK, false);
      setLEDRangeGroup(LedRange.MIDDLE_RIGHT, LightColours.BLACK, LightColours.BLACK, false);
    }
  }

  private void updateOperatorOverrideIndicator() {
    setLEDRangeGroup(LedRange.MIDDLE_MIDDLE, 
      RobotContainer.isOperatorOverride ? LightColours.GREEN : LightColours.BLACK, 
      LightColours.BLACK, 
      false);
  }

  // Helper methods for LED patterns
  private void setCoralScoringPattern(LedRange range, boolean isRightSide) {
    LightColours color = isRightSide ? LightColours.PURPLE : LightColours.YELLOW;
    setLEDRangeGroup(range, color, LightColours.WHITE, false);
  }

  private void setElevatorLevelPattern(ElevatorLevel level, boolean isCoralLoaded) {
    LedRange leftRange = null;
    LedRange rightRange = null;
    LightColours color = isCoralLoaded ? LightColours.WHITE : LightColours.DARKGREEN;

    switch (level) {
      case L1:
        leftRange = LedRange.L1;
        rightRange = LedRange.R1;
        break;
      case L2:
        leftRange = LedRange.L2;
        rightRange = LedRange.R2;
        break;
      case L3:
        leftRange = LedRange.L3;
        rightRange = LedRange.R3;
        break;
      case L4:
        leftRange = LedRange.LEFT_SIDE_FULL;
        rightRange = LedRange.RIGHT_SIDE_FULL;
        break;
      default:
        setLEDRangeGroup(LedRange.LEFT_SIDE_FULL, LightColours.BLACK, LightColours.BLACK, false);
        setLEDRangeGroup(LedRange.RIGHT_SIDE_FULL, LightColours.BLACK, LightColours.BLACK, false);
        return;
    }

    setLEDRangeGroup(leftRange, color, LightColours.WHITE, false);
    setLEDRangeGroup(rightRange, color, LightColours.WHITE, false);
  }
} 