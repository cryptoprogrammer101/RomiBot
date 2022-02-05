// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutonomousSquare extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousSquare(Drivetrain drivetrain, OnBoardIO io) {
    addCommands(
        new DriveStraightGyro(0.5, 5, drivetrain),
        new TurnDegreesGyroPID(90, drivetrain),
        new DriveStraightGyro(0.5, 5, drivetrain),
        new TurnDegreesGyroPID(0.5, drivetrain),
        new DriveStraightGyro(0.5, 5, drivetrain),
        new TurnDegreesGyroPID(0.5, drivetrain),
        new DriveStraightGyro(0.5, 5, drivetrain),
        new TurnDegreesGyroPID(0.5, drivetrain),
        new InstantCommand(
          () -> {
            io.setRedLed(false);
            io.setYellowLed(false);
            io.setGreenLed(true);
          }, io),
        new WaitCommand(3),
        new InstantCommand(
          () -> {
            io.setGreenLed(false);
            io.setYellowLed(false);
            io.setRedLed(true);
          }, io),
        new WaitCommand(3),
        new InstantCommand(
          () -> {
            io.setGreenLed(false);
            io.setRedLed(false);
            io.setYellowLed(true);
          }, io),
        new WaitCommand(3),
        new InstantCommand(
          () -> {
            io.setGreenLed(false);
            io.setRedLed(false);
            io.setYellowLed(false);
          }, io)
        );
  }
}
