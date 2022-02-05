// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveStraightGyro extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_speed;
  private final double m_distance;
  /** Creates a new DriveStraight. */
  public DriveStraightGyro(double speed, double distance, Drivetrain drive) {
    m_distance = distance;
    m_speed = speed;
    m_drive = drive;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
    m_drive.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = -m_drive.getGyroAngleZ();
    double turn = Constants.GAIN * error;
    SmartDashboard.putNumber("Error", -error);
    SmartDashboard.putNumber("Speed", m_speed);
    SmartDashboard.putNumber("Turn", turn);
    m_drive.arcadeDrive(m_speed, turn);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_drive.getAverageDistanceInch()) >= m_distance;
  }
}
