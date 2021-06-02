// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import org.strykeforce.swerve.SwerveModule;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

public class WheelWrapper implements SwerveModule {
    
    CANSparkMax m_driveMotor;
    CANEncoder m_driveEncoder;

    CANSparkMax m_angleMotor;
    CANEncoder m_angleEncoder;

    private double m_azimuthZero = 0.0;
    private double k_deadZone = 1.0;

    Translation2d m_location; // (distance from center, angle)

    public WheelWrapper(CANSparkMax driveMotor, CANSparkMax angleMotor, CANEncoder driveEncoder, CANEncoder angleEncoder, Translation2d location) {
        m_driveMotor = driveMotor;
        m_angleMotor = angleMotor;
        m_driveEncoder = driveEncoder;
        m_angleEncoder = angleEncoder;

        m_location = location;
    }

    public void setWheelAngle(double angle) {
        if(normalizeAngle(getState().angle.getDegrees() - angle) < k_deadZone) {
            m_angleMotor.set(0.5);
        } else if (normalizeAngle(getState().angle.getDegrees() - angle) > k_deadZone) {
            m_angleMotor.set(-0.5);
        } else {
            m_angleMotor.set(0);
        }
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
        m_driveMotor.set(speed / 60 / Constants.k_RPStoMPS);
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
        return new SwerveModuleState(m_driveEncoder.getVelocity() * 60 * Constants.k_RPStoMPS, Rotation2d.fromDegrees(112/360*(m_angleEncoder.getPosition() - m_azimuthZero)));
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
        m_azimuthZero = m_angleEncoder.getPosition();
        // SmartDashboard.putNumber("Azimuth Zero", m_angleEncoder.getPosition());
    }

    @Override
    public void loadAndSetAzimuthZeroReference() {
        // m_angleEncoder.setPosition(SmartDashboard.getNumber("Azimuth Zero", 0));
    }
}