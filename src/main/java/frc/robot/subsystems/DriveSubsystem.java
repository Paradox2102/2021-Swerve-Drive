// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.strykeforce.swerve.SwerveDrive;
import org.strykeforce.swerve.SwerveModule;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.WheelWrapper;

public class DriveSubsystem extends SubsystemBase {

  double k_robotLength = Constants.k_robotLength;
  double k_robotWidth = Constants.k_robotWidth;

  CANSparkMax m_TLDriveMotor = new CANSparkMax(1, MotorType.kBrushless);
  CANEncoder m_TLDriveEncoder = m_TLDriveMotor.getEncoder();
  CANSparkMax m_TLAngleMotor = new CANSparkMax(2, MotorType.kBrushless);
  CANEncoder m_TLAngleEncoder = m_TLAngleMotor.getEncoder();

  CANSparkMax m_TRDriveMotor = new CANSparkMax(3, MotorType.kBrushless);
  CANEncoder m_TRDriveEncoder = m_TRDriveMotor.getEncoder();
  CANSparkMax m_TRAngleMotor = new CANSparkMax(4, MotorType.kBrushless);
  CANEncoder m_TRAngleEncoder = m_TRAngleMotor.getEncoder();

  CANSparkMax m_BLDriveMotor = new CANSparkMax(7, MotorType.kBrushless);
  CANEncoder m_BLDriveEncoder = m_BLDriveMotor.getEncoder();
  CANSparkMax m_BLAngleMotor = new CANSparkMax(8, MotorType.kBrushless);
  CANEncoder m_BLAngleEncoder = m_BLAngleMotor.getEncoder();

  CANSparkMax m_BRDriveMotor = new CANSparkMax(5, MotorType.kBrushless);
  CANEncoder m_BRDriveEncoder = m_BRDriveMotor.getEncoder();
  CANSparkMax m_BRAngleMotor = new CANSparkMax(6, MotorType.kBrushless);
  CANEncoder m_BRAngleEncoder = m_BRAngleMotor.getEncoder();

  SwerveModule[] m_wheels = new SwerveModule[4];

  WheelWrapper m_TLWheel = new WheelWrapper("TL", m_TLDriveMotor, m_TLAngleMotor, m_TLDriveEncoder, m_TLAngleEncoder,
      Constants.k_TLLocation);
  WheelWrapper m_TRWheel = new WheelWrapper("TR", m_TRDriveMotor, m_TRAngleMotor, m_TRDriveEncoder, m_TRAngleEncoder,
      Constants.k_TRLocation);
  WheelWrapper m_BLWheel = new WheelWrapper("BL", m_BLDriveMotor, m_BLAngleMotor, m_BLDriveEncoder, m_BLAngleEncoder,
      Constants.k_BLLocation);
  WheelWrapper m_BRWheel = new WheelWrapper("BR", m_BRDriveMotor, m_BRAngleMotor, m_BRDriveEncoder, m_BRAngleEncoder,
      Constants.k_BRLocation);

  // SwerveDrive swerve = getSwerve();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_TLWheel.loadAndSetAzimuthZeroReference();
    m_TRWheel.loadAndSetAzimuthZeroReference();
    m_BRWheel.loadAndSetAzimuthZeroReference();
    m_BLWheel.loadAndSetAzimuthZeroReference();
  }

  // public void drive(double forward, double strafe, double yaw) {
  //   swerve.drive(forward, strafe, yaw, true);
  // }

  public void setAllAzimuthZero() {
    m_TLWheel.storeAzimuthZeroReference();
    m_TRWheel.storeAzimuthZeroReference();
    m_BRWheel.storeAzimuthZeroReference();
    m_BLWheel.storeAzimuthZeroReference();
  }

  public void setPower(double power) {
    m_TLDriveMotor.set(power);
    m_TRDriveMotor.set(power);
    m_BLDriveMotor.set(power);
    m_BRDriveMotor.set(power);
  }

  public void setAngle(double angle) {
    // m_TLWheel.storeAzimuthZeroReference();
    m_TLWheel.setWheelAngle(angle);
  }

  @Override
  public void periodic() {
    m_TLWheel.periodic();
    m_TRWheel.periodic();
    m_BLWheel.periodic();
    m_BRWheel.periodic();

    // double[] velocity = { m_TLWheel.getState().speedMetersPerSecond, m_TRWheel.getState().speedMetersPerSecond,
    //     m_BLWheel.getState().speedMetersPerSecond, m_BRWheel.getState().speedMetersPerSecond};

    // SmartDashboard.putNumberArray("Velocity", velocity);

    SmartDashboard.putNumber("TL Velocity", m_TLWheel.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("TR Velocity", m_TRWheel.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("BL Velocity", m_BLWheel.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("BR Velocity", m_BRWheel.getState().speedMetersPerSecond);
  }

  public SwerveDrive getSwerve() {

    m_wheels[0] = m_TLWheel;

    return new SwerveDrive(m_wheels);
  }
}
