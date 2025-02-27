// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;

public class IntakeArm extends SubsystemBase {

  private SparkMax m_armSparkMax;
  private SparkMaxSim m_armSparkMaxSim;
  private DutyCycleEncoder m_ArmEncoder;
  private DutyCycleEncoderSim m_ArmEncoderSim;
  private double m_armAngleOffset = IntakeConstants.armAngleOffset;
  public ProfiledPIDController m_PidController;
  private ArmFeedforward m_Feedforward;
  private double kS = IntakeConstants.kS;
  private double kG = IntakeConstants.kG;
  private double kV = IntakeConstants.kV;
  private double kA = IntakeConstants.kA;
  private double kMaxArmAngle = IntakeConstants.MaxArmAngle;
  private double kMinArmAngle = IntakeConstants.MinArmAngle;
  private double kMaxAcceleration = IntakeConstants.kMaxAcceleration;
  private double kMaxVelocity = IntakeConstants.kMaxVelocity;
  private final double kGearRatio = 100.0;
  private final double kVelocityConversionFactor = (360.0 / 60.0) / kGearRatio; // converts RPM to deg/sec
  private final double kPositionConversionFactor = 360.0 / kGearRatio; // converts Revolutions to degrees
  private SparkMaxConfig sparkConfig;
  
  // simulation constants
  private final double kMoment = SingleJointedArmSim.estimateMOI(0.355, 9.1);
  private final double kArmLength = .355;

  private LinearFilter m_velocityFilter;
  private double m_lastArmAngle;
  private double m_armVelocity = 0.0;
  private double pid = 0;
  private double ff = 0;

  private SingleJointedArmSim m_sim;

  /** Creates a new IntakeArm. */
  public IntakeArm() {

    sparkConfig = new SparkMaxConfig();
    sparkConfig.idleMode(IdleMode.kBrake)
               .smartCurrentLimit(50);
    sparkConfig.encoder.positionConversionFactor(kPositionConversionFactor) 
                       .velocityConversionFactor(kVelocityConversionFactor);
    m_armSparkMax = new SparkMax(Constants.CanIds.intakeArmMotorID, MotorType.kBrushed);
    m_armSparkMax.configure(sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_ArmEncoder = new DutyCycleEncoder(IntakeConstants.armEncoderDInput);
    m_ArmEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
    m_PidController = new ProfiledPIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, 
                                                new Constraints(kMaxVelocity, kMaxAcceleration));
    resetFeedForward();
    m_velocityFilter = LinearFilter.singlePoleIIR(0.1, Robot.kDefaultPeriod);
    m_lastArmAngle = getArmAngleDegrees();

    m_ArmEncoderSim = new DutyCycleEncoderSim(m_ArmEncoder);
    DCMotor plant = DCMotor.getAndymark9015(1);
    m_armSparkMaxSim = new SparkMaxSim(m_armSparkMax, plant);
    m_sim = new SingleJointedArmSim(plant,
                                    kGearRatio,
                                    kMoment,
                                    kArmLength,
                                    Math.toRadians(kMinArmAngle),
                                    Math.toRadians(kMaxArmAngle),
                                    true,
                                    0
                                    );
    m_sim.setState( 0, 0);


  }

  private void resetFeedForward() {
    m_Feedforward = new ArmFeedforward(kS, kG, kV, kA);
  }

  private void resetConstraints(){
    m_PidController.setConstraints(new Constraints(kMaxVelocity, kMaxAcceleration));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double newAngle = getArmAngleDegrees();
    double vel = (newAngle - m_lastArmAngle) / Robot.kDefaultPeriod;
    m_armVelocity = m_velocityFilter.calculate(vel);
    m_lastArmAngle = newAngle;

    pid = m_PidController.calculate(newAngle);
    State intermediate = m_PidController.getSetpoint();
    ff = m_Feedforward.calculate(Math.toRadians(intermediate.position),
                                        Math.toRadians(intermediate.velocity));

    m_armSparkMax.set( (pid + ff));
  }

  public void setPositionAngleSetpoint(double angle) {
    double setpoint = MathUtil.clamp(angle, kMinArmAngle, kMaxArmAngle);
    m_PidController.setGoal(setpoint);
  }

  public boolean atSetpoint(){
    return m_PidController.atSetpoint();
  }

  public double getPositionAngleSetpoint() {
    return m_PidController.getGoal().position;
  }

  public double getArmAngleDegrees() {
    return (m_ArmEncoder.get() * 360.0) + m_armAngleOffset;
  }

  public double getArmAngleVelocity() {
    return m_armVelocity;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("ArmAngle", this::getArmAngleDegrees, null);
    builder.addDoubleProperty("ArmAngleOffset", () -> m_armAngleOffset, (x) -> {
      m_armAngleOffset = x;
    });
    builder.addDoubleProperty("kS", () -> kS, (x) -> {
      kS = x;
      resetFeedForward();
    });
    builder.addDoubleProperty("kG", () -> kG, (x) -> {
      kG = x;
      resetFeedForward();
    });
    builder.addDoubleProperty("kV", () -> kV, (x) -> {
      kV = x;
      resetFeedForward();
    });
    builder.addDoubleProperty("kA", () -> kA, (x) -> {
      kA = x;
      resetFeedForward();
    });
    builder.addDoubleProperty("kMaxVelocity", () -> kMaxVelocity, (x) -> {
      kMaxVelocity = x;
      resetConstraints();
    });
    builder.addDoubleProperty("kMaxAcceleration", () -> kMaxAcceleration, (x) -> {
      kMaxAcceleration = x;
      resetConstraints();
    });
    builder.addDoubleProperty("kMinArmAngle", () -> kMinArmAngle, (x) -> {
      kMinArmAngle = x;
    });
    builder.addDoubleProperty("kMaxArmAngle", () -> kMaxArmAngle, (x) -> {
      kMaxArmAngle = x;
    });
    builder.addStringProperty("pid", ()->String.format("%.2f", pid), null);
    builder.addStringProperty("ff", ()->String.format("%.2f", ff), null);
    builder.addDoubleProperty("motorVoltage", ()->m_armSparkMax.getAppliedOutput()*12, null);
  }

  @Override
  public void simulationPeriodic(){
    m_sim.setInputVoltage(m_armSparkMaxSim.getAppliedOutput() * 12.0);
    m_sim.update(Robot.kDefaultPeriod);
    m_armSparkMaxSim.iterate(Math.toDegrees(m_sim.getVelocityRadPerSec()), 12.0, Robot.kDefaultPeriod);
    m_ArmEncoderSim.set(m_sim.getAngleRads()/6.28318);
  }
}
