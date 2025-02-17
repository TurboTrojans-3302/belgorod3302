// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.PrimitiveIterator;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeArm extends SubsystemBase {

  private VictorSPX m_armSpx;
  private DutyCycleEncoder m_ArmEncoder;
  private double m_armAngleOffset = IntakeConstants.armAngleOffset;
  private PIDController m_PidController;
  private double kP = IntakeConstants.kP;
  private double kI = IntakeConstants.kI;
  private double kD = IntakeConstants.kD;

  private LinearFilter m_velocityFilter;
  private double m_lastArmAngle;
  private double m_armVelocity = 0.0;

  /** Creates a new IntakeArm. */
  public IntakeArm() {
    m_armSpx = new VictorSPX(IntakeConstants.intakeArmMotorID);
    m_armSpx.setNeutralMode(NeutralMode.Brake);
    m_armSpx.setInverted(true);
    m_ArmEncoder = new DutyCycleEncoder(IntakeConstants.armEncoderDInput);
    m_ArmEncoder.setDutyCycleRange(1.0/1025.0, 1024.0/1025.0);
    m_PidController = new PIDController(kP, kI, kD);
    m_velocityFilter = LinearFilter.singlePoleIIR(0.1, Robot.kDefaultPeriod);
    m_lastArmAngle = getArmAngleDegrees();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double newAngle = getArmAngleDegrees();
    double vel = (newAngle = m_lastArmAngle) / Robot.kDefaultPeriod;
    m_armVelocity = m_velocityFilter.calculate(vel);
    m_lastArmAngle = newAngle;

    double x = m_PidController.calculate(newAngle);

    m_armSpx.set(VictorSPXControlMode.PercentOutput, x);
  }

  public void setPositionAngleSetpoint(double angle){
    double setpoint = MathUtil.clamp(angle, IntakeConstants.MinArmAngle, IntakeConstants.MaxArmAngle);
    m_PidController.setSetpoint(setpoint);
  }

  public boolean atSetpoint(){
    return m_PidController.atSetpoint();
  }

  public double getPositionAngleSetpoint(){
    return m_PidController.getSetpoint();
  }

  public double getArmAngleDegrees(){
    return (m_ArmEncoder.get() * 360.0) + m_armAngleOffset;
  }

  public double getArmAngleVelocity(){
    return m_armVelocity;
  }

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("ArmAngle", this::getArmAngleDegrees, null);
    builder.addDoubleProperty("ArmAngleOffset", ()->m_armAngleOffset , (x)->{m_armAngleOffset = x;});
  }
}
