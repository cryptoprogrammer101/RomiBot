// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveStraightGyroPID extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_speed;
  private final double m_distance;

  private final PIDController m_controller = new PIDController(Constants.GAIN, 0, 0);

  /** Creates a new DriveStraight. */
  public DriveStraightGyroPID(double speed, double distance, Drivetrain drive) {
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

    SmartDashboard.putNumber("Gyro", getGyroAngleZ());

    // restrict pid values
    double pidVal = MathUtil.clamp(getPID(), -m_speed, m_speed);

    // set pid tolerance to turning
    m_controller.setTolerance(Constants.SPEEDTOLERANCE, Constants.VELOCITYTOLERANCE);

    m_drive.arcadeDrive(m_speed, pidVal);
    
  }

  // return gyro value
  public double getGyroAngleZ() {
    return Math.abs(m_drive.getGyroAngleZ());
  }

  // return pid value
  public double getPID() {
    return m_controller.calculate(getGyroAngleZ(), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
    m_drive.resetGyro();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_drive.getAverageDistanceInch()) >= m_distance;
  }
}
