// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.data.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class TestElevatorAuto extends SequentialCommandGroup {
  private double minLeaderCurrent = Double.MAX_VALUE;
  private double maxLeaderCurrent = Double.MIN_VALUE;
  private double totalLeaderCurrent = 0;
  private int leaderSampleCount = 0;

  private double minFollowerCurrent = Double.MAX_VALUE;
  private double maxFollowerCurrent = Double.MIN_VALUE;
  private double totalFollowerCurrent = 0;
  private int followerSampleCount = 0;

  // NetworkTables publishers
  private final NetworkTable testTable;
  private final DoublePublisher minLeaderCurrentNT;
  private final DoublePublisher maxLeaderCurrentNT;
  private final DoublePublisher avgLeaderCurrentNT;
  private final DoublePublisher minFollowerCurrentNT;
  private final DoublePublisher maxFollowerCurrentNT;
  private final DoublePublisher avgFollowerCurrentNT;

  /**
   * Creates a new TestElevatorAuto command.
   * This command moves the elevator from its minimum height to maximum height and back.
   * @param elevator The elevator subsystem
   */
  public TestElevatorAuto(Elevator elevator) {
    addRequirements(elevator);

    // Initialize NetworkTables publishers
    testTable = NetworkTableInstance.getDefault().getTable("ElevatorTest");
    minLeaderCurrentNT = testTable.getDoubleTopic("Leader Min Current (Amps)").publish();
    maxLeaderCurrentNT = testTable.getDoubleTopic("Leader Max Current (Amps)").publish();
    avgLeaderCurrentNT = testTable.getDoubleTopic("Leader Avg Current (Amps)").publish();
    minFollowerCurrentNT = testTable.getDoubleTopic("Follower Min Current (Amps)").publish();
    maxFollowerCurrentNT = testTable.getDoubleTopic("Follower Max Current (Amps)").publish();
    avgFollowerCurrentNT = testTable.getDoubleTopic("Follower Avg Current (Amps)").publish();

    // Create a command to monitor current during movement
    Command monitorCurrent = Commands.run(() -> {
      double leaderCurrent = elevator.getLeaderCurrent();
      double followerCurrent = elevator.getFollowerCurrent();

      // Update leader stats
      minLeaderCurrent = Math.min(minLeaderCurrent, leaderCurrent);
      maxLeaderCurrent = Math.max(maxLeaderCurrent, leaderCurrent);
      totalLeaderCurrent += leaderCurrent;
      leaderSampleCount++;

      // Update follower stats
      minFollowerCurrent = Math.min(minFollowerCurrent, followerCurrent);
      maxFollowerCurrent = Math.max(maxFollowerCurrent, followerCurrent);
      totalFollowerCurrent += followerCurrent;
      followerSampleCount++;
    });

    // Create a command to publish final statistics
    Command publishStats = Commands.runOnce(() -> {
      minLeaderCurrentNT.set(minLeaderCurrent);
      maxLeaderCurrentNT.set(maxLeaderCurrent);
      avgLeaderCurrentNT.set(totalLeaderCurrent / leaderSampleCount);
      
      minFollowerCurrentNT.set(minFollowerCurrent);
      maxFollowerCurrentNT.set(maxFollowerCurrent);
      avgFollowerCurrentNT.set(totalFollowerCurrent / followerSampleCount);
    });

    // Add commands to the sequential command group
    addCommands(
      // Zero the elevator first to ensure we start from a known position
      new WaitCommand(0.5), // Small delay before starting
      
      // Move to minimum height (should already be there after zeroing)
      new WaitCommand(0.5),
      Commands.runOnce(() -> elevator.setElevatorSetpoint(ElevatorConstants.MIN_ELEVATOR_HEIGHT)),
      Commands.parallel(
        new WaitUntilCommand(() -> elevator.isElevatorAtSetpoint()),
        monitorCurrent
      ),
      
      // Move to maximum height
      new WaitCommand(0.5),
      Commands.runOnce(() -> elevator.setElevatorSetpoint(ElevatorConstants.MAX_ELEVATOR_HEIGHT)),
      Commands.parallel(
        new WaitUntilCommand(() -> elevator.isElevatorAtSetpoint()),
        monitorCurrent
      ),
      
      // Move back to minimum height
      new WaitCommand(0.5),
      Commands.runOnce(() -> elevator.setElevatorSetpoint(ElevatorConstants.MIN_ELEVATOR_HEIGHT)),
      Commands.parallel(
        new WaitUntilCommand(() -> elevator.isElevatorAtSetpoint()),
        monitorCurrent
      ),

      // Publish final statistics
      publishStats
    );
  }
} 