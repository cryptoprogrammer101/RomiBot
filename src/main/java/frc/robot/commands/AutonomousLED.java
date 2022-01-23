// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutonomousLED extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousLED(Drivetrain drivetrain, OnBoardIO io) {
    addCommands(
        new InstantCommand(
          () -> {
            io.setGreenLed(true);
            io.setRedLed(true);
            io.setYellowLed(true);
          }, io),
        new WaitCommand(3),
        new DriveDistance(0.5, 20, drivetrain),
        new TurnDegrees(0.5, 90, drivetrain),
        new DriveDistance(0.5, 20, drivetrain),
        new TurnDegrees(0.5, 90, drivetrain),
        new DriveDistance(0.5, 20, drivetrain),
        new TurnDegrees(0.5, 90, drivetrain),
        new DriveDistance(0.5, 20, drivetrain),
        new TurnDegrees(0.5, 90, drivetrain),
        new InstantCommand(
          () -> {
            io.setGreenLed(false);
            io.setRedLed(false);
            io.setYellowLed(false);
          }, io)
    );
  }
}
