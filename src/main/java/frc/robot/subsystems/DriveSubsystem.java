// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.strykeforce.swerve.SwerveDrive;
import org.strykeforce.swerve.SwerveModule;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.WheelWrapper;

public class DriveSubsystem extends SubsystemBase {

  double k_robotLength = Constants.k_robotLength;
  double k_robotWidth = Constants.k_robotWidth;

  CANSparkMax m_TLDriveMotor = new CANSparkMax(0, MotorType.kBrushless);
  CANEncoder m_TLDriveEncoder = m_TLDriveMotor.getEncoder();
  CANSparkMax m_TLAngleMotor = new CANSparkMax(0, MotorType.kBrushless);
  CANEncoder m_TLAngleEncoder = m_TLAngleMotor.getEncoder();

  CANSparkMax m_TRDriveMotor = new CANSparkMax(0, MotorType.kBrushless);
  CANEncoder m_TRDriveEncoder = m_TRDriveMotor.getEncoder();
  CANSparkMax m_TRAngleMotor = new CANSparkMax(0, MotorType.kBrushless);
  CANEncoder m_TRAngleEncoder = m_TRAngleMotor.getEncoder();

  CANSparkMax m_BLDriveMotor = new CANSparkMax(0, MotorType.kBrushless);
  CANEncoder m_BLDriveEncoder = m_BLDriveMotor.getEncoder();
  CANSparkMax m_BLAngleMotor = new CANSparkMax(0, MotorType.kBrushless);
  CANEncoder m_BLAngleEncoder = m_BLAngleMotor.getEncoder();

  CANSparkMax m_BRDriveMotor = new CANSparkMax(0, MotorType.kBrushless);
  CANEncoder m_BRDriveEncoder = m_BRDriveMotor.getEncoder();
  CANSparkMax m_BRAngleMotor = new CANSparkMax(0, MotorType.kBrushless);
  CANEncoder m_BRAngleEncoder = m_BRAngleMotor.getEncoder();

  SwerveDrive swerve = getSwerve();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public SwerveDrive getSwerve() {
    SwerveModule[] wheels = new SwerveModule[4];

    wheels[0] = (SwerveModule) new WheelWrapper(m_TLDriveMotor, m_TLAngleMotor, m_TLDriveEncoder, m_TLAngleEncoder, Constants.k_TLLocation);

    return new SwerveDrive(wheels);
  }
}
