// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controls;
import frc.robot.RobotContainer;
import frc.robot.data.Constants;
import frc.robot.subsystems.DynamicPathing.DynamicPathingSituation;
import frc.robot.subsystems.groundsuperstructure.GroundIntakeSuperstructure.GroundIntakeSuperstructureState;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;

/**
 * Simple LED subsystem using array-based approach for reliable, conflict-free operation.
 * 
 * Features:
 * - Static colors for elevator levels and states  
 * - Blinking for active processes (intake, etc.)
 * - Rainbow and flow animations
 * - Diagnostic indicators
 * - Predictable "last call wins" behavior
 */
public class Lights extends SubsystemBase {
  // Hardware constants
  private static final int LED_COUNT = 186;
  private static final int FLOW_LENGTH = 15;  // Reduced from 32 for fewer chunks
  private static final double ANIMATION_UPDATE_RATE = 0.05; // 20Hz
  
  private static final CANdle candle = new CANdle(Constants.CANIds.CANdle);
  
  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  // LED STATE
  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  
  private int[] currentLEDs = new int[LED_COUNT];    // Current frame (RGB packed as 0xRRGGBB)
  private int[] lastSentLEDs = new int[LED_COUNT];   // Last sent frame (for change detection)
  
  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  // ANIMATION STATE  
  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  
  private static final Timer animationTimer = new Timer();
  private int flowPosition = 8;                      // Global flow position counter
  private int rainbowOffset = 0;                     // Global rainbow hue offset
  
  private Set<LedRange> rainbowRanges = new HashSet<>();  // Ranges showing rainbow
  private Set<LedRange> flowRanges = new HashSet<>();     // Ranges showing flow
  
  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  // BLINK STATE
  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  
  private long lastBlinkTime = 0;
  private boolean blinkState = false;
  private boolean isCoralIntakeRunning = false;

  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  // LED RANGES - Physical layout of LED strip sections
  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  
  public enum LedRange {
    // Hardware candle LEDs
    CANDLE(0, 8),
    
    // Full sections (non-overlapping)
    LEFT_SIDE_FULL(8, 67),      // 59 LEDs - left side elevator
    MIDDLE_FULL(67, 128),       // 61 LEDs - middle pathfinding indicators  
    RIGHT_SIDE_FULL(128, 186),  // 58 LEDs - right side elevator

    // Progressive elevator ranges (left side - overlapping by design)
    L1(8, 23),                  // Bottom level
    L2(8, 38),                  // Bottom + lower middle  
    L3(8, 52),                  // Bottom + lower + upper middle

    // Progressive elevator ranges (right side - overlapping by design)  
    R1(170, 186),               // Bottom level (16 LEDs, close to L1's 15)
    R2(155, 186),               // Bottom + lower middle (31 LEDs, close to L2's 30)
    R3(141, 186),               // Bottom + lower + upper middle (45 LEDs, close to L3's 44)

    // Middle subsections
    MIDDLE_LEFT(67, 87),        // Pathfinding indicator left
    MIDDLE_MIDDLE(87, 107),     // Override/score indicators  
    MIDDLE_RIGHT(107, 128);     // Pathfinding indicator right

    private final int start;
    private final int end;

    LedRange(int start, int end) {
      this.start = start;
      this.end = end;
    }

    public int getStart() { return start; }
    public int getEnd() { return end; }
  }


  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  // COLORS - RGB color palette  
  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  
  public enum LightColours {
    // Basic colors
    BLACK(0, 0, 0),
    WHITE(255, 255, 255),
    GRAY(127, 127, 127),
    
    // Primary colors
    RED(255, 0, 0),
    GREEN(0, 255, 0), 
    BLUE(0, 0, 255),
    
    // Secondary colors
    YELLOW(255, 190, 0),
    ORANGE(255, 18, 0),
    PURPLE(150, 0, 255),
    CYAN(0, 255, 179),
    PINK(255, 0, 255),
    
    // Specialized colors
    DARKGREEN(21, 102, 13),        // Algae loaded indicator
    LIGHTGREEN(130, 247, 119),     // Light green variant
    LIGHTRED(255, 105, 105),       // Light red variant  
    LIGHTBLUE(103, 120, 214),      // Light blue variant
    NAVY(9, 15, 79),               // Dark blue
    
    // Dimmed variants for elevator indication
    DIM_YELLOW(64, 48, 0),         // 25% brightness yellow
    DIM_PURPLE(38, 0, 64),         // 25% brightness purple
    
    // Uncommon colors
    BROWN(96, 32, 8),
    INFRARED(50, 0, 0),
    SUN(255, 60, 0),
    LIME(187, 255, 0),
    ULTRAVIOLET(50, 0, 100),
    MAGENTA(150, 15, 92),
    
    // Animation colors
    FLOW_COLOR(255, 190, 0);       // Yellow flow animation

    public final int red, green, blue;
    public final int packed;           // RGB packed as 0xRRGGBB for efficiency

    LightColours(int red, int green, int blue) {
      this.red = red;
      this.green = green;
      this.blue = blue;
      this.packed = (red << 16) | (green << 8) | blue;
    }
  }


  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  // CONSTRUCTOR & PERIODIC
  // ═══════════════════════════════════════════════════════════════════════════════════════════════

  public Lights() {
    // Validate LED ranges at startup
    validateLedRanges();
    
    // Configure CANdle hardware  
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.GRB;
    config.brightnessScalar = 0.75;
    config.vBatOutputMode = VBatOutputMode.On;
    config.v5Enabled = true;
    candle.configAllSettings(config, 1000);

    clearHardwareAnimations();
    
    animationTimer.start();
    Arrays.fill(currentLEDs, 0);
    Arrays.fill(lastSentLEDs, 0);
  }
  
  /**
   * Validate that all LED ranges are within bounds
   */
  private void validateLedRanges() {
    for (LedRange range : LedRange.values()) {
      if (range.getStart() < 0 || range.getEnd() > LED_COUNT || range.getStart() >= range.getEnd()) {
        System.err.println("WARNING: Invalid LED range " + range.name() + 
                         " [" + range.getStart() + "-" + range.getEnd() + 
                         "] exceeds LED_COUNT=" + LED_COUNT);
      }
    }
  }

  @Override
  public void periodic() {
    if (RobotBase.isSimulation()) return;

    // Clear frame and update animations
    Arrays.fill(currentLEDs, 0);
    
    if (animationTimer.get() > ANIMATION_UPDATE_RATE) {
      updateAnimations();
      animationTimer.reset();
    }
    
    updateBlinkState();

    // Apply LED logic based on robot state
    if (DriverStation.isEnabled()) {
      handleEnabledState();
    } else {
      handleDisabledState();
    }

    sendLEDsToHardware();
  }

  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  // CORE LED CONTROL METHODS
  // ═══════════════════════════════════════════════════════════════════════════════════════════════

  /**
   * Set a range of LEDs to a specific color
   */
  private void setRange(LedRange range, LightColours color) {
    setRange(range.getStart(), range.getEnd(), color.packed);
  }

  /**
   * Set a range of LEDs with RGB values
   */
  private void setRange(int start, int end, int r, int g, int b) {
    setRange(start, end, (r << 16) | (g << 8) | b);
  }

  /**
   * Set a range of LEDs with packed RGB value
   */
  private void setRange(int start, int end, int packedColor) {
    if (start < 0 || end < 0 || start > LED_COUNT) return;
    int safeStart = Math.max(0, start);
    int safeEnd = Math.min(end, LED_COUNT);
    for (int i = safeStart; i < safeEnd; i++) {
      currentLEDs[i] = packedColor;
    }
  }

  /**
   * Set a range to blink between two colors based on current blink state
   */
  private void setRangeBlinking(LedRange range, LightColours color1, LightColours color2) {
    setRange(range, blinkState ? color1 : color2);
  }

  /**
   * Set a range to display rainbow spectrum
   */
  private void setRangeRainbow(LedRange range) {
    int start = range.getStart();
    int end = range.getEnd();
    int length = end - start;
    
    if (length <= 0 || start < 0 || end > LED_COUNT) return;
    
    for (int i = 0; i < length; i++) {
      int ledIndex = start + i;
      if (ledIndex >= LED_COUNT) break;
      int hue = ((i + rainbowOffset) * 255 / length) % 255;
      int[] rgb = hsvToRgb(hue, 255, 200);
      currentLEDs[ledIndex] = (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
    }
  }

  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  // ANIMATION UPDATES
  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  
  /**
   * Update animation state (rainbow offset, flow position)
   */
  private void updateAnimations() {
    rainbowOffset = (rainbowOffset + 3) % 255;
    
    // Flow animation increments continuously
    // Use a large number that's coprime with typical range lengths to avoid sync issues
    flowPosition = (flowPosition + 2) % 10000;
  }

  /**
   * Update blink state based on time
   */
  private void updateBlinkState() {
    long currentTime = System.currentTimeMillis();
    if (currentTime - lastBlinkTime > 100) {
      blinkState = !blinkState;
      lastBlinkTime = currentTime;
    }
  }

  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  // HARDWARE COMMUNICATION
  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  
  /**
   * Send LED data to hardware efficiently (only send changed LEDs)
   */
  private void sendLEDsToHardware() {
    int start = -1;
    
    for (int i = 0; i < LED_COUNT; i++) {
      if (currentLEDs[i] != lastSentLEDs[i]) {
        if (start == -1) {
          start = i;
        }
      } else {
        if (start != -1) {
          sendBatch(start, i);
          start = -1;
        }
      }
    }
    
    if (start != -1) {
      sendBatch(start, LED_COUNT);
    }
    
    System.arraycopy(currentLEDs, 0, lastSentLEDs, 0, LED_COUNT);
  }

  /**
   * Send a batch of LEDs to hardware efficiently
   */
  private void sendBatch(int start, int end) {
    int batchSize = end - start;
    
    // Send in chunks to avoid overwhelming CAN bus
    // CANdle can handle larger batches, so we'll send in groups
    int maxChunkSize = 10;  // Send 10 LEDs at a time
    
    for (int chunkStart = start; chunkStart < end; chunkStart += maxChunkSize) {
      int chunkEnd = Math.min(chunkStart + maxChunkSize, end);
      int chunkSize = chunkEnd - chunkStart;
      
      // For uniform colors, we can use a single setLEDs call
      // Check if all LEDs in this chunk are the same color
      boolean uniformColor = true;
      int firstColor = currentLEDs[chunkStart];
      for (int i = chunkStart + 1; i < chunkEnd; i++) {
        if (currentLEDs[i] != firstColor) {
          uniformColor = false;
          break;
        }
      }
      
      if (uniformColor && chunkSize > 1) {
        // All LEDs in chunk are same color, send as single command
        int r = (firstColor >> 16) & 0xFF;
        int g = (firstColor >> 8) & 0xFF;
        int b = firstColor & 0xFF;
        candle.setLEDs(r, g, b, 0, chunkStart, chunkSize);
      } else {
        // Different colors, send individually but in quick succession
        for (int i = chunkStart; i < chunkEnd; i++) {
          int color = currentLEDs[i];
          int r = (color >> 16) & 0xFF;
          int g = (color >> 8) & 0xFF;
          int b = color & 0xFF;
          candle.setLEDs(r, g, b, 0, i, 1);
        }
      }
    }
  }

  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  // ROBOT STATE HANDLERS
  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  
  private void handleEnabledState() {
    flowRanges.clear(); // Flow animation only during disabled
    rainbowRanges.clear(); // Clear rainbow ranges before setting new ones
    
    if (!RobotContainer.isOperatorOverride) {
      handleAutomaticElevatorLights();
    } else {
      handleManualElevatorLights();
      enableManualModeRainbow();
    }
    
    updatePathingIndicators();
    updateOverrideIndicators();
    
    // Apply rainbow to active ranges
    for (LedRange range : rainbowRanges) {
      setRangeRainbow(range);
    }
  }

  private void updateOverrideIndicators() {
    if (RobotContainer.isOperatorOverride) {
      setRange(LedRange.MIDDLE_MIDDLE, LightColours.GREEN);
    } else if (Controls.doNotScore.getAsBoolean()) {
      setRange(LedRange.MIDDLE_MIDDLE, LightColours.RED);
    } else {
      setRange(LedRange.MIDDLE_MIDDLE, LightColours.BLACK);
    }
  }

  private void handleDisabledState() {
    clearHardwareAnimations();
    updateDiagnosticIndicators();
    
    // Enable flow animation across entire strip when disabled
    flowRanges.clear();
    flowRanges.add(LedRange.LEFT_SIDE_FULL);
    flowRanges.add(LedRange.MIDDLE_FULL);
    flowRanges.add(LedRange.RIGHT_SIDE_FULL);
    
    // Apply animations to active ranges
    for (LedRange range : flowRanges) {
      setRangeFlow(range);
    }
    for (LedRange range : rainbowRanges) {
      setRangeRainbow(range);
    }
  }
  
  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  // ANIMATION IMPLEMENTATIONS
  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  
  /**
   * Set a range to display flow animation
   */
  private void setRangeFlow(LedRange range) {
    int start = range.getStart();
    int end = range.getEnd();
    int length = end - start;
    
    if (length <= 0 || start < 0 || end > LED_COUNT) return;
    
    // Simple single flow segment that moves smoothly through the range
    // Use a larger modulo to prevent stuttering at wrap-around
    int cycleLength = length + FLOW_LENGTH; // Add flow length to create smooth wrap
    int relativeFlowPos = flowPosition % cycleLength;
    
    // Draw the flow segment
    for (int i = 0; i < FLOW_LENGTH; i++) {
      int ledPos = relativeFlowPos + i - FLOW_LENGTH; // Start behind to allow smooth entry
      
      // Wrap around within the range
      if (ledPos >= 0 && ledPos < length) {
        int ledIndex = start + ledPos;
        
        if (ledIndex >= start && ledIndex < end) {
          // Simple solid yellow for the flow
          currentLEDs[ledIndex] = LightColours.FLOW_COLOR.packed;
        }
      }
    }
  }


  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  // DIAGNOSTIC & STATUS INDICATORS
  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  
  /**
   * Indicators used to perform systems check
   */
  private void updateDiagnosticIndicators() {
    setRange(0, 1, RobotContainer.intakeSubsystem.isAlgaeLoaded() ?
             LightColours.DARKGREEN.packed : LightColours.BLACK.packed);
    
    setRange(1, 2, RobotContainer.intakeSubsystem.isCoralLoaded() ?
             LightColours.WHITE.packed : LightColours.BLACK.packed);

    double pivotPosition = RobotContainer.superstructure.pivot.getPivotPosition();
    setRange(2, 3, Math.abs(pivotPosition) <= 2.0 ?
             LightColours.BLUE.packed : LightColours.BLACK.packed);

    double elevatorPosition = RobotContainer.superstructure.elevator.getElevatorPositionMeters();
    setRange(3, 4, Math.abs(elevatorPosition) <= 0.02 ?
             LightColours.CYAN.packed : LightColours.BLACK.packed);

    setRange(4, 5, RobotContainer.driveSubsystem.limelightsSeeTag() ?
             LightColours.PINK.packed : LightColours.BLACK.packed);

    var alliance = DriverStation.getAlliance();
    LightColours allianceColor = LightColours.BLUE;
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      allianceColor = LightColours.RED;
    }
    setRange(6, 8, allianceColor.packed);
  }

  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  // ELEVATOR LIGHT CONTROL
  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  
  /**
   * Updates elevator side lights based on targeted elevator height in automatic mode
   */
  private void handleAutomaticElevatorLights() {
    if (RobotContainer.isHeadingLockedToL1.getAsBoolean()) {
      setRangeBlinking(LedRange.LEFT_SIDE_FULL, LightColours.ORANGE, LightColours.BLACK);
      setRangeBlinking(LedRange.RIGHT_SIDE_FULL, LightColours.ORANGE, LightColours.BLACK);
      return;
    }
    if (RobotContainer.groundSuperstructure.isL1Ready()) {
      setRange(LedRange.LEFT_SIDE_FULL, LightColours.GREEN);
      setRange(LedRange.RIGHT_SIDE_FULL, LightColours.GREEN);
      return;
    }
    if (RobotContainer.groundSuperstructure.getState() == GroundIntakeSuperstructureState.INTAKE_L1_STATE) {
      setRangeBlinking(LedRange.LEFT_SIDE_FULL, LightColours.GREEN, LightColours.BLACK);
      setRangeBlinking(LedRange.RIGHT_SIDE_FULL, LightColours.GREEN, LightColours.BLACK);
      return;
    }

    SuperstructureState scoringLevel = RobotContainer.dynamicPathingSubsystem.getCoralScoringLevel();
    boolean isRightSide = RobotContainer.dynamicPathingSubsystem.getCoralScoringSide();
    
    switch (scoringLevel) {
      case L1:
        setElevatorSideLights(LedRange.L1, LedRange.R1, isRightSide);
        break;
      case L2:
        setElevatorSideLights(LedRange.L2, LedRange.R2, isRightSide);
        break;
      case L3:
        setElevatorSideLights(LedRange.L3, LedRange.R3, isRightSide);
        break;
      case L4:
        setElevatorSideLights(LedRange.LEFT_SIDE_FULL, LedRange.RIGHT_SIDE_FULL, isRightSide);
        break;
      default:
        break;
    }
  }

  /**
   * Set elevator side lights with appropriate colors based on scoring side
   * Accounts for left/right inversion when robot is on opposite side of reef
   */
  private void setElevatorSideLights(LedRange leftRange, LedRange rightRange, boolean isRightSide) {
    // Get the closest reef face angle to determine if we need to invert left/right
    Pose2d robotPose = RobotContainer.driveSubsystem.getRobotPose();
    Rotation2d closestFaceAngle = RobotContainer.dynamicPathingSubsystem.calculateClosestFaceAngle(robotPose);
    
    // Check if left/right should be inverted based on reef face orientation
    // This mirrors the logic from DynamicPathing.getNearestReefLocationStatic()
    double angleDifference = closestFaceAngle.plus(Rotation2d.k180deg).minus(Rotation2d.kZero).getDegrees();
    boolean shouldInvertSides = Math.abs(angleDifference) > 90;
    
    boolean actualRightSide = shouldInvertSides ? !isRightSide : isRightSide;
    
    // Clear both full sides first
    setRange(LedRange.LEFT_SIDE_FULL, LightColours.BLACK);
    setRange(LedRange.RIGHT_SIDE_FULL, LightColours.BLACK);
    
    if (actualRightSide) {
      // Show rainbow on right side (selected), yellow on left side (inactive)
      setRange(leftRange, LightColours.YELLOW);
      // Set the partial range to yellow first, then add rainbow on top
      setRange(rightRange, LightColours.BLACK);
      rainbowRanges.add(rightRange);
    } else {
      // Show rainbow on left side (selected), yellow on right side (inactive)
      // Set the partial range to yellow first, then add rainbow on top
      setRange(leftRange, LightColours.BLACK);
      rainbowRanges.add(leftRange);
      setRange(rightRange, LightColours.YELLOW);
    }
  }

  private void handleManualElevatorLights() {
    SuperstructureState elevatorLevel =  RobotContainer.superstructure.elevator.getElevatorSetpointEnum();
    boolean hasCoralLoaded = RobotContainer.intakeSubsystem.isCoralLoaded();
    setElevatorLevelPattern(elevatorLevel, hasCoralLoaded);
  }

  private void setElevatorLevelPattern(SuperstructureState level, boolean isCoralLoaded) {
    LightColours color = isCoralLoaded ? LightColours.WHITE : LightColours.BLACK;

    switch (level) {
      case L1:
      case PROCESSOR:
        setRange(LedRange.L1, color);
        setRange(LedRange.R1, color);
        break;
      case L2:
      case ALGAE_L1:
        setRange(LedRange.L2, color);
        setRange(LedRange.R2, color);
        break;
      case L3:
      case ALGAE_L2:
        setRange(LedRange.L3, color);
        setRange(LedRange.R3, color);
        break;
      case L4:
      case NET:
        setRange(LedRange.LEFT_SIDE_FULL, color);
        setRange(LedRange.RIGHT_SIDE_FULL, color);
        break;
      default:
        setRange(LedRange.LEFT_SIDE_FULL, LightColours.BLACK);
        setRange(LedRange.RIGHT_SIDE_FULL, LightColours.BLACK);
        break;
    }
  }

  private void updatePathingIndicators() {
    var pathingSituation = RobotContainer.dynamicPathingSubsystem.getCurrentPathingSituation();
    
    LightColours color = LightColours.BLACK;
    boolean shouldBlink = false;

    if (isCoralIntakeRunning) {
      color = LightColours.WHITE;
      shouldBlink = true;
    } else if (pathingSituation == DynamicPathingSituation.REEF_CORAL) {
      color = LightColours.WHITE;
    } else if (pathingSituation == DynamicPathingSituation.REEF_ALGAE) {
      color = LightColours.DARKGREEN;
    } else if (pathingSituation == DynamicPathingSituation.PROCESSOR) {
      color = LightColours.BLUE;
    } else if (pathingSituation == DynamicPathingSituation.NET) {
      color = LightColours.PINK;
    }

    if (shouldBlink) {
      setRangeBlinking(LedRange.MIDDLE_LEFT, color, LightColours.BLACK);
      setRangeBlinking(LedRange.MIDDLE_RIGHT, color, LightColours.BLACK);
    } else {
      setRange(LedRange.MIDDLE_LEFT, color);
      setRange(LedRange.MIDDLE_RIGHT, color);
    }
  }

  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  // PUBLIC API
  // ═══════════════════════════════════════════════════════════════════════════════════════════════


  public void setCoralIntakeRunning(boolean running) {
    isCoralIntakeRunning = running;
  }

  public void celebrationMode() {
    rainbowRanges.add(LedRange.MIDDLE_FULL);
  }

  public void clearCelebrationMode() {
    rainbowRanges.remove(LedRange.MIDDLE_FULL);
  }

  public void setRainbowAnimation(LedRange range, boolean enabled) {
    if (enabled && range != null) {
      rainbowRanges.add(range);
    } else if (range != null) {
      rainbowRanges.remove(range);
    }
  }

  public void setFlowAnimation(LedRange range, boolean enabled) {
    if (enabled && range != null) {
      flowRanges.add(range);
    } else if (range != null) {
      flowRanges.remove(range);
    }
  }

  public void clearAllLEDs() {
    Arrays.fill(currentLEDs, 0);
    Arrays.fill(lastSentLEDs, 0);
    candle.setLEDs(0, 0, 0, 0, 0, LED_COUNT);
    rainbowRanges.clear();
    flowRanges.clear();
  }

  public void clearHardwareAnimations() {
    candle.animate(null);
    for (int i = 0; i < candle.getMaxSimultaneousAnimationCount(); i++) {
      candle.clearAnimation(i);
    }
  }

  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  // MANUAL MODE RAINBOW
  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  
  private void enableManualModeRainbow() {
    SuperstructureState elevatorLevel = RobotContainer.superstructure.elevator.getElevatorSetpointEnum();
    boolean hasCoralLoaded = RobotContainer.intakeSubsystem.isCoralLoaded();
    
    if (!hasCoralLoaded) {
      switch (elevatorLevel) {
        case L1:
        case PROCESSOR:
          rainbowRanges.add(LedRange.L1);
          rainbowRanges.add(LedRange.R1);
          break;
        case L2:
        case ALGAE_L1:
          rainbowRanges.add(LedRange.L2);
          rainbowRanges.add(LedRange.R2);
          break;
        case L3:
        case ALGAE_L2:
          rainbowRanges.add(LedRange.L3);
          rainbowRanges.add(LedRange.R3);
          break;
        case L4:
        case NET:
          rainbowRanges.add(LedRange.LEFT_SIDE_FULL);
          rainbowRanges.add(LedRange.RIGHT_SIDE_FULL);
          break;
        default:
          break;
      }
    }
  }

  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  // COLOR UTILITIES
  // ═══════════════════════════════════════════════════════════════════════════════════════════════
  
  private int[] hsvToRgb(int h, int s, int v) {
    // Normalize hue to 0-360 range, then convert to 0-6 range
    double hNorm = (h % 255) * 360.0 / 255.0;
    double hh = hNorm / 60.0;
    int i = (int)hh % 6; // Ensure i is always 0-5
    double ff = hh - (int)hh;
    double p = v * (1.0 - s / 255.0);
    double q = v * (1.0 - (s / 255.0) * ff);
    double t = v * (1.0 - (s / 255.0) * (1.0 - ff));
    
    switch(i) {
      case 0: return new int[]{v, (int)Math.round(t), (int)Math.round(p)};
      case 1: return new int[]{(int)Math.round(q), v, (int)Math.round(p)};
      case 2: return new int[]{(int)Math.round(p), v, (int)Math.round(t)};
      case 3: return new int[]{(int)Math.round(p), (int)Math.round(q), v};
      case 4: return new int[]{(int)Math.round(t), (int)Math.round(p), v};
      case 5:
      default: return new int[]{v, (int)Math.round(p), (int)Math.round(q)};
    }
  }
} 