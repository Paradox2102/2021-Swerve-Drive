// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import org.strykeforce.swerve.SwerveModule;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WheelWrapper implements SwerveModule {
    
    CANSparkMax m_driveMotor;
    CANEncoder m_driveEncoder;

    CANSparkMax m_angleMotor;
    CANEncoder m_angleEncoder;

    private double m_azimuthZero = 0.0;
    private double k_deadZone = 1.0;
    private double m_angle = 0;
    private String m_name;
    
    final String m_smartDashboardName;

    private double k_p = 0.01;

    Translation2d m_location; // (distance from center, angle)

    public WheelWrapper(String name, CANSparkMax driveMotor, CANSparkMax angleMotor, CANEncoder driveEncoder, CANEncoder angleEncoder, Translation2d location) {
        m_driveMotor = driveMotor;
        m_angleMotor = angleMotor;
        m_driveEncoder = driveEncoder;
        m_angleEncoder = angleEncoder;
        m_name = name;
        m_smartDashboardName = m_name + " Azimuth Zero";

        m_location = location;
    }

    public void periodic() {
        double error = normalizeAngle(getState().angle.getDegrees() - m_angle);
        double power = -error * k_p;
        if (Math.abs(error) < k_deadZone) {
            power = 0;
        }
        m_angleMotor.set(power);
        // System.out.println(error + " " + m_angle + " " + k_deadZone);
    }

    public void setWheelAngle(double angle) {
        m_angle = angle;
    }

    public double normalizeAngle(double angle) {
        double a = angle % 360;

        if (a > 180)
        {
            a -= 360;
        }
        else if (a < -180)
        {
            a += 360;
        }

        return a;
    }

    public void setSpeed(double speed) {
        m_driveMotor.getPIDController().setReference(speed / Constants.k_RPMtoMPS, ControlType.kVelocity);
    }

    // public double normalize(double value) {
    //     return (value+180)%360-180;
    // }

    @Override
    public double getMaxSpeedMetersPerSecond() {
        return Constants.k_maxSpeed;
    }

    @Override
    public Translation2d getWheelLocationMeters() {
        return m_location; 
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getVelocity() * Constants.k_RPMtoMPS, Rotation2d.fromDegrees(360.0/112*(m_angleEncoder.getPosition() - m_azimuthZero)));
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isDriveOpenLoop) {
        setWheelAngle(SwerveModuleState.optimize(desiredState, getState().angle).angle.getDegrees());
        setSpeed(desiredState.speedMetersPerSecond);
    }

    @Override
    public void resetDriveEncoder() {
        m_driveEncoder.setPosition(0);
    }

    @Override
    public void storeAzimuthZeroReference() {
        SmartDashboard.setPersistent(m_smartDashboardName);

        m_azimuthZero = m_angleEncoder.getPosition();
        SmartDashboard.putNumber(m_smartDashboardName, m_angleEncoder.getPosition());
        // System.out.println(SmartDashboard.getNumber(m_smartDashboardName, 0));
        SmartDashboard.setPersistent(m_smartDashboardName);
    }

    @Override
    public void loadAndSetAzimuthZeroReference() {
        m_azimuthZero = SmartDashboard.getNumber(m_smartDashboardName, 0);
        SmartDashboard.putNumber(m_smartDashboardName, m_azimuthZero);
    }
}