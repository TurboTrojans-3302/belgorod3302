// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems.Eddie;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystemBase;

/**
 * todos
 * 
 * confirm absolute and relative encoders are CCW+
 * calibrate the angle offsets
 * 
 */

public class EddieDriveTrain extends DriveSubsystemBase {

    private double maxSpeed = DriveConstants.kMaxSpeedMetersPerSecond; // m/s
    private double maxRotation = DriveConstants.kMaxRotation;

    private final SwerveDriveKinematics kinematics = DriveConstants.kinematics;

    private static final double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians((329.59 - 360));
    private static final double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(207.0);
    private static final double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(54.58 - 180);
    private static final double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(19.07);
    private static final double kPgain = 0.080;
    private static final double kDgain = 0;

    private static EddieDriveTrain m_instance;

    ModuleConfiguration rightSideConfiguration = new ModuleConfiguration(
            0.10033,
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
            true,
            (9.0 / 24.0) * (14.0 / 72.0),
            false);

    ModuleConfiguration leftSideConfiguration = new ModuleConfiguration(
            0.10033,
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
            false,
            (9.0 / 24.0) * (14.0 / 72.0),
            false);

    private final TTSwerveModule frontLeftModule = new TTSwerveModule(
            leftSideConfiguration,
            DriveConstants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
            DriveConstants.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR,
            DriveConstants.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER,
            FRONT_LEFT_ANGLE_OFFSET);
    private final TTSwerveModule frontRightModule = new TTSwerveModule(
            rightSideConfiguration,
            DriveConstants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
            DriveConstants.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR,
            DriveConstants.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER,
            FRONT_RIGHT_ANGLE_OFFSET);

    private final TTSwerveModule backLeftModule = new TTSwerveModule(
            leftSideConfiguration,
            DriveConstants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR,
            DriveConstants.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR,
            DriveConstants.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER,
            BACK_LEFT_ANGLE_OFFSET);

    private final TTSwerveModule backRightModule = new TTSwerveModule(
            rightSideConfiguration,
            DriveConstants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR,
            DriveConstants.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR,
            DriveConstants.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER,
            BACK_RIGHT_ANGLE_OFFSET);

    private final AHRS ahrs = new AHRS(SerialPort.Port.kUSB);

    private Timer stillTime = new Timer();

    public EddieDriveTrain() {
        m_instance = this;

        ahrs.reset();
        calibrateSterrRelativeEncoder();

        // todo add the swerve drive to the dashboard
        // SmartDashboard.putData("Swerve Drive", new Sendable() {
        // @Override
        // public void initSendable(SendableBuilder builder) {
        // builder.setSmartDashboardType("SwerveDrive");

        // builder.addDoubleProperty("Front Left Angle", () ->
        // frontLeftModule.getSteerAngle(), null);
        // builder.addDoubleProperty("Front Left Velocity", () ->
        // frontLeftModule.getDriveVelocity(), null);

        // builder.addDoubleProperty("Front Right Angle", () ->
        // frontRightModule.getSteerAngle(), null);
        // builder.addDoubleProperty("Front Right Velocity", () ->
        // frontRightModule.getDriveVelocity(), null);

        // builder.addDoubleProperty("Back Left Angle", () ->
        // backLeftModule.getSteerAngle(), null);
        // builder.addDoubleProperty("Back Left Velocity", () ->
        // backLeftModule.getDriveVelocity(), null);

        // builder.addDoubleProperty("Back Right Angle", () ->
        // backRightModule.getSteerAngle(), null);
        // builder.addDoubleProperty("Back Right Velocity", () ->
        // backRightModule.getDriveVelocity(), null);

        // builder.addDoubleProperty("Robot Angle", () -> getGyroAngleRadians(), null);
        // }
        // });

    }

    public static EddieDriveTrain getInstance() {
        if (m_instance == null) {
            m_instance = new EddieDriveTrain();
        }

        return m_instance;
    }

    @Override
    public void periodic() {
        setMaxSpeed();

        if (Math.abs(getSpeed()) > 1e-6 && Math.abs(getTurnRate()) > 1e-6) {
            stillTime.restart();
        }

    }

    public double turnToHeadingDegrees(double headingDegrees) {
        double angle = getGyroAngleDegrees();
        double currentAngularRate = getAngularRateDegPerSec();
        double angle_error = angleDeltaDeg(headingDegrees, angle);
        double yawCommand = -angle_error * kPgain - (currentAngularRate) * kDgain;
        return yawCommand;
    }

    /*
     * Drive the robot, relative to the field.
     * All parameters are [-1, 1]
     * 
     * @param x speed in the "north" direction, forward if you're on the blue side
     * 
     * @param y speed in the "east" direction, left if you're on the blue side
     * 
     * @param rotation turn speed, positive is counter-clockwise
     * 
     */
    public void driveFieldOriented(Double x, Double y, Double rotation) {
        Translation2d translation = new Translation2d(x, y);
        translation = translation.times(maxSpeed);
        rotation *= maxRotation;
        driveFieldOriented(translation, rotation);
    }

    public void driveRobotOriented(Double x, Double y, Double rotation) {
        Translation2d translation = new Translation2d(x, y);
        translation = translation.times(maxSpeed);
        rotation *= maxRotation;
        driveRobotOriented(translation, rotation);
    }

    private double speedToVoltage(double speed) {
        return MathUtil.clamp(speed / maxSpeed, -1.0, 1.0) * 12.0;
    }

    public void drive(ChassisSpeeds speeds) {

        SwerveModuleState[] states = DriveConstants.kinematics.toSwerveModuleStates(speeds);
        frontLeftModule.set(speedToVoltage(states[0].speedMetersPerSecond), states[0].angle.getRadians());
        frontRightModule.set(speedToVoltage(states[1].speedMetersPerSecond), states[1].angle.getRadians());
        backLeftModule.set(speedToVoltage(states[2].speedMetersPerSecond), states[2].angle.getRadians());
        backRightModule.set(speedToVoltage(states[3].speedMetersPerSecond), states[3].angle.getRadians());

    }

    public void testSetAll(double voltage, double angleRadians) {
        frontLeftModule.set(voltage, angleRadians);
        frontRightModule.set(voltage, angleRadians);
        backLeftModule.set(voltage, angleRadians);
        backRightModule.set(voltage, angleRadians);
    }

    public double getPitchDeg() {
        return ahrs.getPitch();
    }

    public double getGyroAngleRadians() {
        return MathUtil.angleModulus(Units.degreesToRadians(getGyroAngleDegrees()));
    }

    public double getGyroAngleDegrees() {
        return -ahrs.getAngle();
    }

    public double getTurnRate() {
        return -ahrs.getRate(); // todo confirm the sign of this
    }

    // todo test this! don't think its ever been used
    public void setGyroAngleDeg(double robotangle) {
        double angle2 = -robotangle;
        double err = angle2 - ahrs.getAngle();
        double newAdj = err + ahrs.getAngleAdjustment();
        ahrs.setAngleAdjustment(newAdj);
    }

    public double getAngularRateDegPerSec() {
        return -ahrs.getRate();
    }

    static public double angleDeltaDeg(double src, double dest) {
        double delta = (dest - src) % 360.0;
        if (Math.abs(delta) > 180) {
            delta = delta - (Math.signum(delta) * 360);
        }
        return delta;
    }

    public void calibrateSterrRelativeEncoder() {
        frontLeftModule.calibrateSterrRelativeEncoder();
        frontRightModule.calibrateSterrRelativeEncoder();
        backLeftModule.calibrateSterrRelativeEncoder();
        backRightModule.calibrateSterrRelativeEncoder();
    }

    public void stop() {
        frontLeftModule.set(0, Math.PI / 4);
        frontRightModule.set(0, -Math.PI / 4);
        backLeftModule.set(0, -Math.PI / 4);
        backRightModule.set(0, Math.PI);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(
                frontLeftModule.getState(),
                frontRightModule.getState(),
                backLeftModule.getState(),
                backRightModule.getState());
    }

    public void setX() {
        stop();
    }

    @Override
    public double getMaxSpeedLimit() {
        return DriveConstants.kMaxSpeedMetersPerSecond;
    }

    @Override
    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] { frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition() };
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("gyroAngleDegrees", this::getGyroAngleDegrees, this::setGyroAngleDeg);
        builder.addDoubleProperty("maxSpeed", ()->{ return maxSpeed; }, (x)->{ maxSpeed = x;});
        builder.addDoubleProperty("maxRotation", ()->{ return maxRotation; }, (x)->{ maxRotation = x;});
    }

}
