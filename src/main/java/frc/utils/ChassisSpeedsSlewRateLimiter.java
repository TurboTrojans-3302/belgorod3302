// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public class ChassisSpeedsSlewRateLimiter {
    private Translation2d lastTranslation;
    private SlewRateLimiter rotationLimiter;
    private double rateLimit;
    private double m_prevTime;

    public ChassisSpeedsSlewRateLimiter(double translateLimit, double rotateLimit) {
        this(translateLimit, rotateLimit, new ChassisSpeeds());
    }

    public ChassisSpeedsSlewRateLimiter(double translateLimit, double rotateLimit, ChassisSpeeds initialValue) {
        this.rateLimit = translateLimit;
        rotationLimiter = new SlewRateLimiter(rotateLimit, -rotateLimit, );
        lastTranslation = new Translation2d(initialValue.vxMetersPerSecond, initialValue.vyMetersPerSecond);
        lastRotation = initialValue.omegaRadiansPerSecond;
        m_prevTime = MathSharedStore.getTimestamp();
    }

    public ChassisSpeeds calculate(ChassisSpeeds input) {
        Translation2d newTranslation = new Translation2d(input.vxMetersPerSecond, input.vyMetersPerSecond);
        Translation2d delta = newTranslation.minus(lastTranslation);
        double magnitude = delta.getNorm();
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - m_prevTime;
        double newMagnitude = MathUtil.clamp(magnitude,0.0, rateLimit * elapsedTime);

        if (magnitude > 1e-6) {
            newTranslation = lastTranslation.plus(delta.times(newMagnitude / magnitude));
        }
        double newRotation = 

        m_prevTime = currentTime;
        lastTranslation = newTranslation;
        return result;
    }
}
