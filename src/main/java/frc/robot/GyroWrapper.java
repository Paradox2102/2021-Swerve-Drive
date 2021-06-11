// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.interfaces.Gyro;

/** Add your docs here. */
public class GyroWrapper implements Gyro {

    PigeonIMU m_gyro;

    public GyroWrapper(PigeonIMU gyro) {
        m_gyro = gyro;
    }

    @Override
    public void close() throws Exception {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void calibrate() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void reset() {
        m_gyro.setYaw(0);
    }

    @Override
    public double getAngle() {
        double[] ypr = new double[3];
        m_gyro.getYawPitchRoll(ypr);
        return ypr[0];
    }

    @Override
    public double getRate() {
        double[] xyz = new double[3];
        m_gyro.getRawGyro(xyz);
        return xyz[2];
    }
}
