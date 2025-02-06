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

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.DriveSubsystemBase;

/**
 *
 */
public class EddieDriveTrain extends DriveSubsystemBase {

    public static final double MAX_SPEED = 5.0; // m/s
    public static final double MAX_ROTATION = 4.0;

    private static final double FRONT_LEFT_ANGLE_OFFSET = Math.toRadians(209.787 - 180.0);
    private static final double FRONT_RIGHT_ANGLE_OFFSET = Math.toRadians(151.43);
    private static final double BACK_LEFT_ANGLE_OFFSET = Math.toRadians(303.485 - 180.0);
    private static final double BACK_RIGHT_ANGLE_OFFSET = Math.toRadians(342.33);
    private static final double kPgain = 0.080;
    private static final double kDgain = 0;

    private LaserCan dxSensor = new LaserCan(DriveConstants.DRIVETRAIN_DX_SENSOR);

    private static EddieDriveTrain m_instance;

    ModuleConfiguration rightSideConfiguration = new ModuleConfiguration(
            0.10033,
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
            true,
            (9.0 / 24.0) * (14.0 / 72.0),
            true);

    ModuleConfiguration leftSideConfiguration = new ModuleConfiguration(
            0.10033,
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
            false,
            (9.0 / 24.0) * (14.0 / 72.0),
            true);

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

    // dashboard stuff
    DoublePublisher speedPub;
    DoublePublisher headingPub;
    DoublePublisher maxSpeedPub;
    DoublePublisher dxPub;
    BooleanPublisher dxGoodPub;

    public EddieDriveTrain() {
        m_instance = this;

        ahrs.reset();

        mOdometry = new SwerveDrivePoseEstimator(
                DriveConstants.kinematics, Rotation2d.fromRadians(getAngleRad()),
                new SwerveModulePosition[] {
                        frontLeftModule.getPosition(),
                        frontRightModule.getPosition(),
                        backLeftModule.getPosition(),
                        backRightModule.getPosition()
                }, defaultStartPosition);

        try {
            dxSensor.setRangingMode(LaserCan.RangingMode.LONG);
            dxSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            dxSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("LaserCan Configuration failed! " + e);
        }

        // dashboard stuff
        NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("DriveTrain");
        speedPub = tagsTable.getDoubleTopic("Speed").publish();
        maxSpeedPub = tagsTable.getDoubleTopic("MaxSpeed").publish();
        headingPub = tagsTable.getDoubleTopic("Heading").publish();
        dxPub = tagsTable.getDoubleTopic("DX Sensor").publish();
        dxGoodPub = tagsTable.getBooleanTopic("DX Good").publish();
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

        // Update the pose
        updateOdometry(new SwerveModulePosition[] {
                frontLeftModule.getPosition(), frontRightModule.getPosition(),
                backLeftModule.getPosition(), backRightModule.getPosition()
        });

        if(Math.abs(getSpeed()) > 1e-6 && Math.abs(getTurnRate()) > 1e-6) {
            stillTime.restart();
        }

        //todo is this necessary? to wait for a still interval before getting a vision estimate?
        if(stillTime.get() > 0.5){
            PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue("limeLight");
            //todo choose the coordinate system (red/blue)
            //todo is this necessary? how often is the estimate invalid?
            if(LimelightHelpers.validPoseEstimate(est)){
                mOdometry.addVisionMeasurement(est.pose, est.timestampSeconds);
            }
        }

        speedPub.set(getSpeed());
        maxSpeedPub.set(getMaxSpeed());
        headingPub.set(getHeading());
        dxGoodPub.set(distanceMeasurmentGood());
        dxPub.set(getDistanceToObjectMeters());
    }

    public double turnToHeading(double heading) {
        double angle = getHeading();
        double currentAngularRate = getAngularRateDegPerSec();
        double angle_error = angleDeltaDeg(heading, angle);
        double yawCommand = -angle_error * kPgain - (currentAngularRate) * kDgain;
        return yawCommand;
    }

    public void driveFieldOriented(Double x, Double y, Double rotation) {
        Translation2d translation = new Translation2d(x, y);
        translation = translation.times(MAX_SPEED);
        rotation *= MAX_ROTATION;
        driveFieldOriented(translation, rotation);
    }

    public void driveRobotOriented(Double x, Double y, Double rotation) {
        Translation2d translation = new Translation2d(x, y);
        translation = translation.times(MAX_SPEED);
        rotation *= MAX_ROTATION;
        driveRobotOriented(translation, rotation);
    }

    private double speedToVoltage(double speed) {
        return MathUtil.clamp(speed / MAX_SPEED, -1.0, 1.0) * 12.0;
    }

    public void drive(ChassisSpeeds speeds) {

        SwerveModuleState[] states = DriveConstants.kinematics.toSwerveModuleStates(speeds);
        frontLeftModule.set(speedToVoltage(states[0].speedMetersPerSecond), states[0].angle.getRadians());
        frontRightModule.set(speedToVoltage(states[1].speedMetersPerSecond), states[1].angle.getRadians());
        backLeftModule.set(speedToVoltage(states[2].speedMetersPerSecond), states[2].angle.getRadians());
        backRightModule.set(speedToVoltage(states[3].speedMetersPerSecond), states[3].angle.getRadians());

    }

    public void setAll(double speed, double angleRadians) {
        frontLeftModule.set(speed, angleRadians);
        frontRightModule.set(speed, angleRadians);
        backLeftModule.set(speed, angleRadians);
        backRightModule.set(speed, angleRadians);
    }

    public void resetGyroscope() {
        ahrs.setAngleAdjustment(ahrs.getAngle() - ahrs.getAngleAdjustment());
    }

    public double getPitchDeg() {
        return ahrs.getPitch();
    }

    public double getGyroAngleRadians() {
        return MathUtil.angleModulus(Units.degreesToRadians(-ahrs.getAngle()));
    }

    public double getTurnRate() {
        return -ahrs.getRate(); // todo confirm the sign of this
    }

    public double getAngleRad() {
        return Math.toRadians(getHeading());
    }

    public void setAngleDeg(double robotangle) {
        double angle2 = -robotangle;
        double err = angle2 - ahrs.getAngle();
        double newAdj = err + ahrs.getAngleAdjustment();
        ahrs.setAngleAdjustment(newAdj);
    }

    public double getAngularRateDegPerSec() {
        return -ahrs.getRate();
    }

    public Translation2d getVelocityVector() {
        ChassisSpeeds chassisSpeeds = getChassisSpeeds();
        return new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
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
        return DriveConstants.kinematics.toChassisSpeeds(
                frontLeftModule.getState(),
                frontRightModule.getState(),
                backLeftModule.getState(),
                backRightModule.getState());
    }

    public double getSpeed() {
        Translation2d velocityVector = getVelocityVector();
        return velocityVector.getNorm();
    }

    public void setX() {
        stop();
    }

    public void resetOdometry(Pose2d pose) {
        mOdometry.resetPosition(
                Rotation2d.fromDegrees(getHeading()),
                new SwerveModulePosition[] {
                        frontLeftModule.getPosition(),
                        frontRightModule.getPosition(),
                        backLeftModule.getPosition(),
                        backRightModule.getPosition()
                },
                pose);
    }

    public Double getDistanceToObjectMeters() {
        Measurement m = dxSensor.getMeasurement();
        return m.distance_mm * 0.001;
    }

    public boolean distanceMeasurmentGood() {
        Measurement m = dxSensor.getMeasurement();
        return m.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
    }

    @Override
    public double getMaxSpeedLimit() {
        return DriveConstants.kMaxSpeedMetersPerSecond;
    }

}
