// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.strykeforce.swerve.SwerveDrive;
import org.strykeforce.swerve.SwerveModule;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GyroWrapper;
import frc.robot.WheelWrapper;

public class DriveSubsystem extends SubsystemBase {

  double k_robotLength = Constants.k_robotLength;
  double k_robotWidth = Constants.k_robotWidth;

  CANSparkMax m_FLDriveMotor = new CANSparkMax(1, MotorType.kBrushless);
  CANEncoder m_FLDriveEncoder = m_FLDriveMotor.getEncoder();
  CANPIDController m_FLController;
  CANSparkMax m_FLAngleMotor = new CANSparkMax(2, MotorType.kBrushless);
  CANEncoder m_FLAngleEncoder = m_FLAngleMotor.getEncoder();

  CANSparkMax m_FRDriveMotor = new CANSparkMax(3, MotorType.kBrushless);
  CANEncoder m_FRDriveEncoder = m_FRDriveMotor.getEncoder();
  CANPIDController m_FRController;
  CANSparkMax m_FRAngleMotor = new CANSparkMax(4, MotorType.kBrushless);
  CANEncoder m_FRAngleEncoder = m_FRAngleMotor.getEncoder();

  CANSparkMax m_BLDriveMotor = new CANSparkMax(7, MotorType.kBrushless);
  CANEncoder m_BLDriveEncoder = m_BLDriveMotor.getEncoder();
  CANPIDController m_BLController;
  CANSparkMax m_BLAngleMotor = new CANSparkMax(8, MotorType.kBrushless);
  CANEncoder m_BLAngleEncoder = m_BLAngleMotor.getEncoder();

  CANSparkMax m_BRDriveMotor = new CANSparkMax(5, MotorType.kBrushless);
  CANEncoder m_BRDriveEncoder = m_BRDriveMotor.getEncoder();
  CANPIDController m_BRController;
  CANSparkMax m_BRAngleMotor = new CANSparkMax(6, MotorType.kBrushless);
  CANEncoder m_BRAngleEncoder = m_BRAngleMotor.getEncoder();

  PigeonIMU m_pigeon = new PigeonIMU(0);

  SwerveModule[] m_wheels = new SwerveModule[4];

  WheelWrapper m_FLWheel = new WheelWrapper("FL", m_FLDriveMotor, m_FLAngleMotor, m_FLDriveEncoder, m_FLAngleEncoder,
      Constants.k_FLLocation);
  WheelWrapper m_FRWheel = new WheelWrapper("FR", m_FRDriveMotor, m_FRAngleMotor, m_FRDriveEncoder, m_FRAngleEncoder,
      Constants.k_FRLocation);
  WheelWrapper m_BLWheel = new WheelWrapper("BL", m_BLDriveMotor, m_BLAngleMotor, m_BLDriveEncoder, m_BLAngleEncoder,
      Constants.k_BLLocation);
  WheelWrapper m_BRWheel = new WheelWrapper("BR", m_BRDriveMotor, m_BRAngleMotor, m_BRDriveEncoder, m_BRAngleEncoder,
      Constants.k_BRLocation);

  Gyro m_gyro = new GyroWrapper(m_pigeon);

  SwerveDrive m_swerve;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_FLController = m_FLDriveMotor.getPIDController();
    m_FRController = m_FRDriveMotor.getPIDController();
    m_BLController = m_BLDriveMotor.getPIDController();
    m_BRController = m_BRDriveMotor.getPIDController();

    setPID(m_FLController, Constants.k_FLF, Constants.k_P, Constants.k_I, Constants.k_iZone);
    setPID(m_FRController, Constants.k_FRF, Constants.k_P, Constants.k_I, Constants.k_iZone);
    setPID(m_BLController, Constants.k_BLF, Constants.k_P, Constants.k_I, Constants.k_iZone);
    setPID(m_BRController, Constants.k_BRF, Constants.k_P, Constants.k_I, Constants.k_iZone);

    m_FLWheel.loadAndSetAzimuthZeroReference();
    m_FRWheel.loadAndSetAzimuthZeroReference();
    m_BRWheel.loadAndSetAzimuthZeroReference();
    m_BLWheel.loadAndSetAzimuthZeroReference();

    m_swerve = getSwerve();
   resetGyro();
  }

  public void drive(double forward, double strafe, double yaw) {
    m_swerve.drive(forward, strafe, yaw, true);
  }

  public void setPID(CANPIDController controller, double f, double p, double i, double iZone) {
    controller.setFF(f);
    controller.setP(p);
    controller.setI(i);
    controller.setIZone(iZone);
    controller.setD(0);
    controller.setOutputRange(-1, 1);
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  public void setBrakeMode(boolean isBreak) {
    if(isBreak) {
      m_FLDriveMotor.setIdleMode(IdleMode.kBrake);
      m_FRDriveMotor.setIdleMode(IdleMode.kBrake);
      m_BLDriveMotor.setIdleMode(IdleMode.kBrake);
      m_BRDriveMotor.setIdleMode(IdleMode.kBrake);
    } else {
      m_FLDriveMotor.setIdleMode(IdleMode.kCoast);
      m_FRDriveMotor.setIdleMode(IdleMode.kCoast);
      m_BLDriveMotor.setIdleMode(IdleMode.kCoast);
      m_BRDriveMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  public void setAllAzimuthZero() {
    m_FLWheel.storeAzimuthZeroReference();
    m_FRWheel.storeAzimuthZeroReference();
    m_BRWheel.storeAzimuthZeroReference();
    m_BLWheel.storeAzimuthZeroReference();
  }

  public double getFLEncoderValue() {
    return m_FLDriveEncoder.getPosition();
  }

  public void setSpeed(double speed) {
    m_FLController.setReference(speed, ControlType.kVelocity);
    m_FRController.setReference(speed, ControlType.kVelocity);
    m_BLController.setReference(speed, ControlType.kVelocity);
    m_BRController.setReference(speed, ControlType.kVelocity);
  }

  public void setPower(double power) {
    m_FLController.setReference(power, ControlType.kDutyCycle);
    m_FRController.setReference(power, ControlType.kDutyCycle);
    m_BLController.setReference(power, ControlType.kDutyCycle);
    m_BRController.setReference(power, ControlType.kDutyCycle);
  }

  public void setAngle(double angle) {
    // m_FLWheel.storeAzimuthZeroReference();
    m_FLWheel.setWheelAngle(angle);
  }

  @Override
  public void periodic() {
    m_FLWheel.periodic();
    m_FRWheel.periodic();
    m_BLWheel.periodic();
    m_BRWheel.periodic();

    // SmartDashboard.putNumber("Velocity FL", m_FLWheel.getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("Velocity FR", m_FRWheel.getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("Velocity BL", m_BLWheel.getState().speedMetersPerSecond);
    // SmartDashboard.putNumber("Velocity BR", m_BRWheel.getState().speedMetersPerSecond);

    // SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
  }

  public SwerveDrive getSwerve() {

    m_wheels[0] = m_FLWheel;
    m_wheels[1] = m_FRWheel;
    m_wheels[2] = m_BLWheel;
    m_wheels[3] = m_BRWheel;

    return new SwerveDrive(m_gyro, m_wheels);
  }
}
