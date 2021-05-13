// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopDriveCommand extends CommandBase {

  DriveSubsystem m_driveSubsystem;

  double k_deadband = Constants.k_deadband;

  DoubleSupplier m_forward;
  DoubleSupplier m_strafe;
  DoubleSupplier m_yaw;

  public TeleopDriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier yaw) {
    m_driveSubsystem = driveSubsystem;

    m_forward = forward;
    m_strafe = strafe;
    m_yaw = yaw;

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double forward = deadband(m_forward.getAsDouble());
    double strafe = deadband(m_strafe.getAsDouble());
    double yaw = deadband(m_yaw.getAsDouble());

    m_driveSubsystem.drive(-forward, -strafe, -yaw);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double deadband(double value) {
    if (Math.abs(value) < k_deadband)
      return 0.0;
    return value;
  }
}
