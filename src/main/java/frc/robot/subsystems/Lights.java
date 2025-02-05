// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.data.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

public class Lights extends SubsystemBase {
  private static final CANdle candle = new CANdle(Constants.CANIds.CANdle); 
  private static final int LED_COUNT = 64;

  private static final Timer blinkTimer = new Timer();
  private boolean isBlinkColour = true;
  public boolean isEndgameWarning = false;
  private double blinkRate = 0.1;
  private int[][] ledColors;
  
  private Map<LedRange, LightColours> ledRangeColours = new EnumMap<>(LedRange.class);

  public enum LedRange {
    CANDLE(0,8),
    RIGHT_SIDE_FULL(8,26),
    MIDDLE_FULL(26,44),
    LEFT_SIDE_FULL(44,64),
    RIGHT_SIDE_TOP(20,26),
    RIGHT_SIDE_BOTTOM(8,20),
    LEFT_SIDE_TOP(44,51),
    LEFT_SIDE_BOTTOM(51,64);

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

    configAll.stripType = LEDStripType.GRB;
    configAll.v5Enabled = true;

    candle.configAllSettings(configAll, 100);

    ledColors = new int[LED_COUNT][3];
  }

  /**
   * This method is called periodically by the CommandScheduler.
   * It updates the LED animations based on the robot's state.
   */
  @Override
  public void periodic() {
    
    if(DriverStation.isDisabled()){
      setLEDRangeGroup(LedRange.RIGHT_SIDE_BOTTOM, LightColours.GREEN, LightColours.GREEN, false);
      /* 
      LedAnimation currentAnimation = getLedAnimation();
      if (shooterSubsystem.isNote()){
        setLEDRangeGroup(LedRange.RIGHT_SIDE_BOTTOM, LightColours.GREEN, LightColours.GREEN, false);
      }
      if (shooterSubsystem.isFullyInNote()){
        setLEDRangeGroup(LedRange.LEFT_SIDE_BOTTOM, LightColours.GREEN, LightColours.GREEN, false);
      }
      if (elevatorSubsystem.getElevatorPosition() < -0.5 || anglerSubsystem.getAnglerDegrees() < -30.0) {
        setLEDRangeGroup(LedRange.MIDDLE_FULL, LightColours.RED, LightColours.RED, false);
      } else if (Math.abs(elevatorSubsystem.getElevatorPosition()) > 0.5 
                 || Math.abs(anglerSubsystem.getAnglerDegrees()) > 1) {
        setLEDRangeGroup(LedRange.MIDDLE_FULL, LightColours.YELLOW, LightColours.GREEN, false);
      } else {
        setLEDRangeGroup(LedRange.MIDDLE_FULL, LightColours.GREEN, LightColours.GREEN, false);
      }*/
      updateBlinkTimer();
      updateLedRanges();
      candle.animate(null);
      //candle.animate(currentAnimation.getAnimation());
    }
    else{
      candle.animate(null);
      updateBlinkTimer();
      updateLedRanges();
    }
  }

  /**
   * Determines the appropriate LED animation based on the robot's state.
   * @return The LedAnimation to be displayed
   */
  private LedAnimation getLedAnimation() {
    /* 
    if (elevatorSubsystem.getElevatorPosition() < -0.5 || anglerSubsystem.getAnglerDegrees() < -30.0) {
        return LedAnimation.STROBE;
    } else if (Math.abs(elevatorSubsystem.getElevatorPosition()) > 0.5 
               || Math.abs(anglerSubsystem.getAnglerDegrees()) > 1) {
        return LedAnimation.LARSON;
    } else {
        
    }*/
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

    // Clear ledRangeColours for the next update cycle
    ledRangeColours.clear();
  }
} 