// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Ludwig;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Robot;

public class MAXSwerveModule implements SwerveModule, Sendable {
  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;
  public final SparkSim m_drivingSparkSim;
  public final SparkSim m_turningSparkSim;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;
  private final SparkAbsoluteEncoderSim m_turningEncoderSim;
  private final SparkAbsoluteEncoderSim m_drivingEncoderSim;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  private double steerP, steerI, steerD;
  private double driveP, driveI, driveD;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);
    m_drivingSparkSim = new SparkSim(m_drivingSpark, DCMotor.getNEO(1));
    m_turningSparkSim = new SparkSim(m_turningSpark, DCMotor.getNeo550(1));
  

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_turningEncoderSim = new SparkAbsoluteEncoderSim(m_turningSpark);
    m_drivingEncoderSim = new SparkAbsoluteEncoderSim(m_drivingSpark);

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
    if(Robot.isSimulation()){
      m_drivingSparkSim.setPosition(0);
      m_drivingEncoderSim.setPosition(0);
    }

    steerP = m_turningSpark.configAccessor.closedLoop.getP();
    steerI = m_turningSpark.configAccessor.closedLoop.getI();
    steerD = m_turningSpark.configAccessor.closedLoop.getD();
    driveP = m_drivingSpark.configAccessor.closedLoop.getP();
    driveI = m_drivingSpark.configAccessor.closedLoop.getI();
    driveD = m_drivingSpark.configAccessor.closedLoop.getD();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset)); 

    // Optimize the reference state to avoid spinning further than 90 degrees.
    //correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));
    // todo put the optimization back

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  // replacement for the original get instance function???
  public static MAXSwerveModule getInstance(int drivingID, int turningID, double chassisAngularOffset) {

    MAXSwerveModule m_swerveModule = new MAXSwerveModule(drivingID, turningID, chassisAngularOffset);

    return m_swerveModule;

  }

  @Override
  public double getDriveVelocity() {
    return m_drivingEncoder.getVelocity();
  }

  @Override
  public double getSteerAngle() {
    return getPosition().angle.getRadians() - m_chassisAngularOffset;
  }

  @Override
  public void set(double voltage, double steerAngleRadians) {
    m_drivingSpark.setVoltage(voltage);
    m_turningClosedLoopController.setReference(steerAngleRadians + m_chassisAngularOffset, ControlType.kPosition);
  }

  public void testSet(double voltage, double angleRadians) {
    this.set(voltage, angleRadians);
  }

  private void setPIDConstants(SparkMax spark, double p, double i, double d) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(p, i, d);
    spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void iterateSim(double dt) {
    m_drivingSparkSim.iterate(m_desiredState.speedMetersPerSecond, RoboRioSim.getVInVoltage(), dt);
    m_drivingEncoderSim.iterate(m_desiredState.speedMetersPerSecond, dt);
    m_turningSparkSim.setPosition(m_desiredState.angle.getRadians());
    m_turningEncoderSim.setPosition(m_desiredState.angle.getRadians());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Swerve Module " + m_drivingSpark.getDeviceId() + "/" + m_turningSpark.getDeviceId());
    builder.addDoubleProperty("AngleOffset", () -> Math.toDegrees(m_chassisAngularOffset), (x) -> {
      m_chassisAngularOffset = Math.toRadians(x);
    });
    builder.addDoubleProperty("Steer P",
        () -> steerP,
        (x) -> {
          steerP = x;
          setPIDConstants(m_turningSpark, steerP, steerI, steerD);
        });
    builder.addDoubleProperty("Steer I",
        () -> steerI,
        (x) -> {
          steerI = x;
          setPIDConstants(m_turningSpark, steerP, steerI, steerD);
        });
    builder.addDoubleProperty("Steer D",
        () -> steerD,
        (x) -> {
          steerD = x;
          setPIDConstants(m_turningSpark, steerP, steerI, steerD);
        });
    builder.addDoubleProperty("Drive P",
        () -> driveP,
        (x) -> {
          driveP = x;
          setPIDConstants(m_drivingSpark, driveP, driveI, driveD);
        });
    builder.addDoubleProperty("Drive I",
        () -> driveI,
        (x) -> {
          driveI = x;
          setPIDConstants(m_drivingSpark, driveP, driveI, driveD);
        });
    builder.addDoubleProperty("Drive D",
        () -> driveD,
        (x) -> {
          driveD = x;
          setPIDConstants(m_drivingSpark, driveP, driveI, driveD);
        });
  }

}