package frc.robot.subsystems.Eddie;

import com.swervedrivespecialties.swervelib.DriveController;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SteerController;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.rev.NeoSteerConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.swervedrivespecialties.swervelib.ctre.CanCoderAbsoluteConfiguration;
import com.swervedrivespecialties.swervelib.ctre.CtreUtils;
import com.revrobotics.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import static com.swervedrivespecialties.swervelib.rev.RevUtils.checkNeoError;


//TODO Confirm that both Relative and Absolute encoders are CCW+

@SuppressWarnings("deprecation")
public class TTSwerveModule implements SwerveModule {

    private static final double DEADBAND = 0.005;
        private SteerControllerImplementation mSteerController;
        private DriveControllerImplementation mDriveController;
        private Mk4ModuleConfiguration mModuleConfiguration;
    
        public TTSwerveModule(
            ModuleConfiguration mechanicalConfiguration,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset
        ){
            mModuleConfiguration = new Mk4ModuleConfiguration();
    
            // make the drive controller. copied from NeoDriveControllerFactoryBuilder
    
            // Setup encoder
            EncoderConfig driveEncoderConfig = new EncoderConfig();
            double positionConversionFactor = Math.PI * mechanicalConfiguration.getWheelDiameter() * mechanicalConfiguration.getDriveReduction();
            driveEncoderConfig.positionConversionFactor(positionConversionFactor);
            driveEncoderConfig.velocityConversionFactor(positionConversionFactor / 60.0);
    
            double nominalVoltage = mModuleConfiguration.getNominalVoltage();
            double currentLimit = mModuleConfiguration.getDriveCurrentLimit();
            SparkMaxConfig driveConfig = new SparkMaxConfig();
            driveConfig.inverted(mechanicalConfiguration.isDriveInverted())
                .voltageCompensation(nominalVoltage)
                .smartCurrentLimit((int)currentLimit)
                .idleMode(IdleMode.kBrake)
                .apply(driveEncoderConfig);
        
            SparkMax driveMotor = new SparkMax(driveMotorPort, MotorType.kBrushless);
            driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);            
        
            RelativeEncoder driveEncoder = driveMotor.getEncoder();
            driveEncoder.setPosition(0);
            mDriveController = new DriveControllerImplementation(driveMotor, driveEncoder);
    
            // create steer controller. copied from NeoSteerControllerFactoryBuilder
    
            CanCoderAbsoluteConfiguration encoderconfig = new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset);
            NeoSteerConfiguration<CanCoderAbsoluteConfiguration> steerConfiguration = new NeoSteerConfiguration<>(steerMotorPort, encoderconfig);
    
    
            CANCoderConfiguration config = new CANCoderConfiguration();
            config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
            config.magnetOffsetDegrees = Math.toDegrees(encoderconfig.getOffset());
            config.sensorDirection = false;
    
            CANCoder encoder = new CANCoder(encoderconfig.getId());
            CtreUtils.checkCtreError(encoder.configAllSettings(config, 250), "Failed to configure CANCoder");
            System.out.println("encoder abs sensor range:" + encoder.configGetAbsoluteSensorRange().toString());
    
            CtreUtils.checkCtreError(encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100, 250), "Failed to configure CANCoder update rate");
    
            EncoderImplementation absoluteEncoder = new EncoderImplementation(encoder);
            absoluteEncoder.setInverted(true);
    
            
            EncoderConfig steerEncoderConfig = new EncoderConfig();
            double steerPositionConversionFactor = 2.0 * Math.PI * mechanicalConfiguration.getSteerReduction();
            steerEncoderConfig.positionConversionFactor(steerPositionConversionFactor);
            steerEncoderConfig.velocityConversionFactor(steerPositionConversionFactor / 60.0);
    
            final double pidProportional = 1.0;
            final double pidIntegral = 0.0;
            final double pidDerivative = 0.1;
    
            ClosedLoopConfig steerPIDConfig = new ClosedLoopConfig();
            steerPIDConfig.p(pidProportional)
                .i(pidIntegral)
                .d(pidDerivative)
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
    
            SparkMaxConfig steerConfig = new SparkMaxConfig();
            steerConfig.inverted(!mechanicalConfiguration.isSteerInverted()) //TODO should this be inverted here?
                .voltageCompensation(nominalVoltage)
                .smartCurrentLimit((int)currentLimit)
                .idleMode(IdleMode.kBrake)
                .apply(steerEncoderConfig)
                .apply(steerPIDConfig);
    
            SparkMax steerMotor = new SparkMax(steerConfiguration.getMotorPort(), MotorType.kBrushless);
            steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
            RelativeEncoder integratedEncoder = steerMotor.getEncoder();
            checkNeoError(integratedEncoder.setPosition(absoluteEncoder.getAbsoluteAngle()), "Failed to set NEO encoder position");
    
            
            
    
            mSteerController = new SteerControllerImplementation(steerMotor, absoluteEncoder);
        }            
    
        public void calibrateSterrRelativeEncoder(){
    
            mSteerController.calibrateRelativeEncoder();
    
        }
    
        private static class DriveControllerImplementation implements DriveController {
            private final SparkMax motor;
            private final RelativeEncoder encoder;
    
            private DriveControllerImplementation(SparkMax motor, RelativeEncoder encoder) {
                this.motor = motor;
                this.encoder = encoder;
            }
    
            @Override
            public void setReferenceVoltage(double voltage) {
                motor.setVoltage(voltage);
            }
    
            @Override
            public double getStateVelocity() {
                return encoder.getVelocity();
            }
        }
        
        
        public static class SteerControllerImplementation implements SteerController {
    
            @SuppressWarnings({"FieldCanBeLocal", "unused"})
            private final SparkMax motor;
            private final RelativeEncoder motorEncoder;
            private final EncoderImplementation absoluteEncoder;
    
            private double referenceAngleRadians = 0;
    
            public SteerControllerImplementation(SparkMax motor, EncoderImplementation absoluteEncoder) {
                this.motor = motor;
                this.motorEncoder = motor.getEncoder();
                this.absoluteEncoder = absoluteEncoder;
            }
    
            @Override
            public double getReferenceAngle() {
                return referenceAngleRadians;
            }
            
            public void calibrateRelativeEncoder(){
                double absoluteAngle = absoluteEncoder.getAbsoluteAngle();
                motorEncoder.setPosition(absoluteAngle);
            }
    
    
    
    
            @Override
            public void setReferenceAngle(double referenceAngleRadians) {
                double currentAngleRadians = motorEncoder.getPosition();
    
                double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
                if (currentAngleRadiansMod < 0.0) {
                    currentAngleRadiansMod += 2.0 * Math.PI;
                }
    
                // The reference angle has the range [0, 2pi) but the Neo's encoder can go above that
                double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
                if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
                    adjustedReferenceAngleRadians -= 2.0 * Math.PI;
                } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
                    adjustedReferenceAngleRadians += 2.0 * Math.PI;
                }
    
                this.referenceAngleRadians = referenceAngleRadians;
    
                motor.getClosedLoopController().setReference(adjustedReferenceAngleRadians, ControlType.kPosition);
            }
    
    
    
            @Override
            public double getStateAngle() {
                double motorAngleRadians = motorEncoder.getPosition();
                motorAngleRadians %= 2.0 * Math.PI;
                if (motorAngleRadians < 0.0) {
                    motorAngleRadians += 2.0 * Math.PI;
                }
    
                return motorAngleRadians;
            }
    
            public double getAbsoluteAngle(){
                return absoluteEncoder.getAbsoluteAngle();
            }
        }
    
        private static class EncoderImplementation implements AbsoluteEncoder {
            private final CANCoder encoder;
    
            private EncoderImplementation(CANCoder encoder) {
                this.encoder = encoder;
            }
    
            public double getAbsoluteAngle() {
                double angle = Math.toRadians(encoder.getAbsolutePosition());
                angle %= 2.0 * Math.PI;
                if (angle < 0.0) {
                    angle += 2.0 * Math.PI;
                }
    
                return angle;
            }
    
            @Override
            public double getPosition() {
                return encoder.getPosition();
            }
    
            @Override
            public double getVelocity() {
                return encoder.getVelocity();
            }
    
            // @Override
            // public REVLibError setPositionConversionFactor(double factor) {
            //     // TODO Auto-generated method stub
            //     return REVLibError.kNotImplemented;
            // }
    
            // @Override
            // public double getPositionConversionFactor() {
            //     // TODO Auto-generated method stub
            //     return 0;
            // }
    
            // @Override
            // public REVLibError setVelocityConversionFactor(double factor) {
            //     // TODO Auto-generated method stub
            //     return REVLibError.kNotImplemented;
            // }
    
            // @Override
            // public double getVelocityConversionFactor() {
            //     // TODO Auto-generated method stub
            //     return 0;
            // }
    
            public REVLibError setInverted(boolean inverted) {
                if( ErrorCode.OK == encoder.configSensorDirection(inverted)){
                    return REVLibError.kOk;
                } else { 
                    return REVLibError.kError;
                }
            }
    
            public boolean getInverted() {
                return encoder.configGetSensorDirection();
            }
    
            // @Override
            // public REVLibError setAverageDepth(int depth) {
            //     // TODO Auto-generated method stub
            //     return REVLibError.kNotImplemented;
            // }
    
            // @Override
            // public int getAverageDepth() {
            //     // TODO Auto-generated method stub
            //     return 0;
            // }
    
            // @Override
            // public REVLibError setZeroOffset(double offset) {
            //     return ErrorCode.OK == encoder.configMagnetOffset(offset) ? REVLibError.kOk : REVLibError.kError;
            // }
    
            // @Override
            // public double getZeroOffset() {
            //     return encoder.configGetMagnetOffset();
            // }
        }
    
        public enum Direction {
            CLOCKWISE,
            COUNTER_CLOCKWISE
        }
    
    
    
    
        @Override
        public double getDriveVelocity() {
            return mDriveController.getStateVelocity();
        }
    
        @Override
        public double getSteerAngle() {
            return mSteerController.getStateAngle();
        }
    
        @Override
        public void set(double driveVoltage, double steerAngle) {
            steerAngle %= (2.0 * Math.PI);
            if (steerAngle < 0.0) {
                steerAngle += 2.0 * Math.PI;
            }
    
            double difference = steerAngle - getSteerAngle();
            // Change the target angle so the difference is in the range [-pi, pi) instead of [0, 2pi)
            if (difference >= Math.PI) {
                steerAngle -= 2.0 * Math.PI;
            } else if (difference < -Math.PI) {
                steerAngle += 2.0 * Math.PI;
            }
            difference = steerAngle - getSteerAngle(); // Recalculate difference
    
            // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
            // movement of the module is less than 90 deg
            if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
                // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
                steerAngle += Math.PI;
                driveVoltage *= -1.0;
            }
    
            // Put the target angle back into the range [0, 2pi)
            steerAngle %= (2.0 * Math.PI);
            if (steerAngle < 0.0) {
                steerAngle += 2.0 * Math.PI;
            }
    
            if(Math.abs(driveVoltage) > DEADBAND ){
                mDriveController.setReferenceVoltage(driveVoltage);
                mSteerController.setReferenceAngle(steerAngle);
            }else{
                mDriveController.setReferenceVoltage(0.0);
            }
    }    

    public double getAbsoluteAngle(){
        return mSteerController.getAbsoluteAngle();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            mDriveController.encoder.getPosition(),
            new Rotation2d(getSteerAngle()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            mDriveController.getStateVelocity(),
            new Rotation2d(getSteerAngle()));
    }
}
