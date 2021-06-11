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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.WheelWrapper;

public class DriveSubsystem extends SubsystemBase {

  double k_robotLength = Constants.k_robotLength;
  double k_robotWidth = Constants.k_robotWidth;

  CANSparkMax m_TLDriveMotor = new CANSparkMax(1, MotorType.kBrushless);
  CANEncoder m_TLDriveEncoder = m_TLDriveMotor.getEncoder();
  CANPIDController m_TLController;
  CANSparkMax m_TLAngleMotor = new CANSparkMax(2, MotorType.kBrushless);
  CANEncoder m_TLAngleEncoder = m_TLAngleMotor.getEncoder();

  CANSparkMax m_TRDriveMotor = new CANSparkMax(3, MotorType.kBrushless);
  CANEncoder m_TRDriveEncoder = m_TRDriveMotor.getEncoder();
  CANPIDController m_TRController;
  CANSparkMax m_TRAngleMotor = new CANSparkMax(4, MotorType.kBrushless);
  CANEncoder m_TRAngleEncoder = m_TRAngleMotor.getEncoder();

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

  PigeonIMU m_gryo = new PigeonIMU(0);

  SwerveModule[] m_wheels = new SwerveModule[4];

  WheelWrapper m_TLWheel = new WheelWrapper("TL", m_TLDriveMotor, m_TLAngleMotor, m_TLDriveEncoder, m_TLAngleEncoder,
      Constants.k_TLLocation);
  WheelWrapper m_TRWheel = new WheelWrapper("TR", m_TRDriveMotor, m_TRAngleMotor, m_TRDriveEncoder, m_TRAngleEncoder,
      Constants.k_TRLocation);
  WheelWrapper m_BLWheel = new WheelWrapper("BL", m_BLDriveMotor, m_BLAngleMotor, m_BLDriveEncoder, m_BLAngleEncoder,
      Constants.k_BLLocation);
  WheelWrapper m_BRWheel = new WheelWrapper("BR", m_BRDriveMotor, m_BRAngleMotor, m_BRDriveEncoder, m_BRAngleEncoder,
      Constants.k_BRLocation);

  SwerveDrive m_swerve;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_TLController = m_TLDriveMotor.getPIDController();
    m_TRController = m_TRDriveMotor.getPIDController();
    m_BLController = m_BLDriveMotor.getPIDController();
    m_BRController = m_BRDriveMotor.getPIDController();

    setPID(m_TLController, Constants.k_TLF, Constants.k_P, Constants.k_I, Constants.k_iZone);
    setPID(m_TRController, Constants.k_TRF, Constants.k_P, Constants.k_I, Constants.k_iZone);
    setPID(m_BLController, Constants.k_BLF, Constants.k_P, Constants.k_I, Constants.k_iZone);
    setPID(m_BRController, Constants.k_BRF, Constants.k_P, Constants.k_I, Constants.k_iZone);

    m_TLWheel.loadAndSetAzimuthZeroReference();
    m_TRWheel.loadAndSetAzimuthZeroReference();
    m_BRWheel.loadAndSetAzimuthZeroReference();
    m_BLWheel.loadAndSetAzimuthZeroReference();

    m_swerve = getSwerve();
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

  public void setBrakeMode(boolean isBreak) {
    if(isBreak) {
      m_TLDriveMotor.setIdleMode(IdleMode.kBrake);
      m_TRDriveMotor.setIdleMode(IdleMode.kBrake);
      m_BLDriveMotor.setIdleMode(IdleMode.kBrake);
      m_BRDriveMotor.setIdleMode(IdleMode.kBrake);
    } else {
      m_TLDriveMotor.setIdleMode(IdleMode.kCoast);
      m_TRDriveMotor.setIdleMode(IdleMode.kCoast);
      m_BLDriveMotor.setIdleMode(IdleMode.kCoast);
      m_BRDriveMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  public void setAllAzimuthZero() {
    m_TLWheel.storeAzimuthZeroReference();
    m_TRWheel.storeAzimuthZeroReference();
    m_BRWheel.storeAzimuthZeroReference();
    m_BLWheel.storeAzimuthZeroReference();
  }

  public double getTLEncoderValue() {
    return m_TLDriveEncoder.getPosition();
  }

  public void setSpeed(double speed) {
    m_TLController.setReference(speed, ControlType.kVelocity);
    m_TRController.setReference(speed, ControlType.kVelocity);
    m_BLController.setReference(speed, ControlType.kVelocity);
    m_BRController.setReference(speed, ControlType.kVelocity);
  }

  public void setPower(double power) {
    m_TLController.setReference(power, ControlType.kDutyCycle);
    m_TRController.setReference(power, ControlType.kDutyCycle);
    m_BLController.setReference(power, ControlType.kDutyCycle);
    m_BRController.setReference(power, ControlType.kDutyCycle);
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

    SmartDashboard.putNumber("Velocity TL", m_TLWheel.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Velocity TR", m_TRWheel.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Velocity BL", m_BLWheel.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Velocity BR", m_BRWheel.getState().speedMetersPerSecond);
  }

  public SwerveDrive getSwerve() {

    m_wheels[0] = m_TLWheel;
    m_wheels[1] = m_TRWheel;
    m_wheels[2] = m_BLWheel;
    m_wheels[3] = m_BRWheel;

    return new SwerveDrive(m_wheels);
  }
}
