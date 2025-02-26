// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXSimCollection;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;

public class IntakeArm extends SubsystemBase {

  private VictorSPX m_armSpx;
  private VictorSPXSimCollection m_armSpxSim;
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
  private final double kGearRatio = 100.0;
  
  // simulation constants
  private final double kMoment = 32.3; //kg-m^2
  private final double kArmLength = .355;

  private LinearFilter m_velocityFilter;
  private double m_lastArmAngle;
  private double m_armVelocity = 0.0;

  private SingleJointedArmSim m_sim;

  /** Creates a new IntakeArm. */
  public IntakeArm() {
    m_armSpx = new VictorSPX(Constants.CanIds.intakeArmMotorID);
    m_armSpx.setNeutralMode(NeutralMode.Brake);
    m_armSpx.setInverted(true);
    m_armSpxSim = m_armSpx.getSimCollection();
    m_ArmEncoder = new DutyCycleEncoder(IntakeConstants.armEncoderDInput);
    m_ArmEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
    m_PidController = new ProfiledPIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, 
                                                new Constraints(IntakeConstants.kMaxVelocity, IntakeConstants.kMaxAcceleration));
    resetFeedForward();
    m_velocityFilter = LinearFilter.singlePoleIIR(0.1, Robot.kDefaultPeriod);
    m_lastArmAngle = getArmAngleDegrees();
    m_sim = new SingleJointedArmSim(DCMotor.getAndymarkRs775_125(1),
                                    kGearRatio,
                                    kMoment,
                                    kArmLength,
                                    Math.toRadians(kMinArmAngle),
                                    Math.toRadians(kMaxArmAngle),
                                    true,
                                    Math.toRadians(kMaxArmAngle)
                                    );
  }

  private void resetFeedForward() {
    m_Feedforward = new ArmFeedforward(kS, kG, kV, kA);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double newAngle = getArmAngleDegrees();
    double vel = (newAngle = m_lastArmAngle) / Robot.kDefaultPeriod;
    m_armVelocity = m_velocityFilter.calculate(vel);
    m_lastArmAngle = newAngle;

    double pid = m_PidController.calculate(newAngle);
    double ff = m_Feedforward.calculate(newAngle, m_armVelocity);

    m_armSpx.set(VictorSPXControlMode.PercentOutput, pid + ff);
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
    builder.addDoubleProperty("kMinArmAngle", () -> kMinArmAngle, (x) -> {
      kMinArmAngle = x;
    });
    builder.addDoubleProperty("kMaxArmAngle", () -> kMaxArmAngle, (x) -> {
      kMaxArmAngle = x;
    });
  }

  @Override
  public void simulationPeriodic(){
    m_sim.setInputVoltage(m_armSpxSim.getMotorOutputLeadVoltage());
    m_sim.update(Robot.kDefaultPeriod);
    m_ArmEncoderSim.set(m_sim.getAngleRads()/6.28318);
  }
}
