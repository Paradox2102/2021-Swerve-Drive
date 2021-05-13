// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

    Translation2d m_location;// (distance from center, angle)

    public WheelWrapper(CANSparkMax driveMotor, CANSparkMax angleMotor, CANEncoder driveEncoder, CANEncoder angleEncoder, Translation2d location) {
        m_driveMotor = driveMotor;
        m_angleMotor = angleMotor;
        m_driveEncoder = driveEncoder;
        m_angleEncoder = angleEncoder;

        m_location = location;
    }

    public void setWheelAngle(double angle) {
        if(angle > getState().angle.getDegrees()%360) {
            m_angleMotor.set(0.5);
        } else if (angle < getState().angle.getDegrees()%360) {
            m_angleMotor.set(-0.5);
        } else {
            m_angleMotor.set(0);
        }
    }

    public void setSpeed(double speed) {
        m_driveMotor.set(speed / 60 / Constants.k_RPStoMPS);
    }

    @Override
    public double getMaxSpeedMetersPerSecond() {
        return 0;
    }

    @Override
    public Translation2d getWheelLocationMeters() {
        return m_location; 
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_angleEncoder.getVelocity() * 60 * Constants.k_RPStoMPS, Rotation2d.fromDegrees(360*(m_angleEncoder.getPosition() - m_azimuthZero)));
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
    }

    @Override
    public void loadAndSetAzimuthZeroReference() {
        // TODO Auto-generated method stub
        
    }
}
