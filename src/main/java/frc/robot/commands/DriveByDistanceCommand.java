// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveByDistanceCommand extends CommandBase {

  DriveSubsystem m_driveSubsystem;
  double m_rotations;
  double m_initValue;

  public DriveByDistanceCommand(DriveSubsystem driveSubsystem, double rotations) {
    m_driveSubsystem = driveSubsystem;
    m_rotations = rotations;

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initValue = m_driveSubsystem.getFLEncoderValue();
    m_driveSubsystem.setSpeed(1200);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_driveSubsystem.getFLEncoderValue() - m_initValue) > m_rotations;
  }
}
