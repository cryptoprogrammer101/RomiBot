// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnDegreesGyro extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_degrees;
  private final double m_speed;

  /**
   * Creates a new TurnDegrees. This command will turn your robot for a desired rotation (in
   * degrees) and rotational speed.
   *
   * @param speed The speed which the robot will drive. Negative is in reverse.
   * @param degrees Degrees to turn. Leverages encoders to compare distance.
   * @param drive The drive subsystem on which this command will run
   */
  public TurnDegreesGyro(double speed, double degrees, Drivetrain drive) {
    m_degrees = Math.abs(degrees);
    m_drive = drive;
    m_speed = speed;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set motors to stop, read encoder values for starting point
    m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
    m_drive.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if gyro is within margin
    if (Math.abs(m_drive.getGyroAngleZ()) + Constants.MARGIN > m_degrees) {
      // slow down
      m_drive.arcadeDrive(0, m_speed * Constants.MULTIPLIER);
    } else {
      // continue turning 
      m_drive.arcadeDrive(0, m_speed);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if gyro is greater than degrees
    return Math.abs(m_drive.getGyroAngleZ()) > m_degrees;
  }

}
