// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;

public class Climbers extends SubsystemBase {
  /** Creates a new Climbers. */
  SparkMax m_rightClimber;
  SparkMax m_leftClimber;
  SparkClosedLoopController m_leftController;
  SparkClosedLoopController m_rightController;
  RelativeEncoder m_leftEncoder;
  RelativeEncoder m_rightEncoder;
  //used for detecting when climber is at highest position
  DigitalInput limitSwitchClimber;
  double kP = ClimberConstants.kP;
  double kI = ClimberConstants.kI;
  double kD = ClimberConstants.kD;
  double kLowerLimit = ClimberConstants.kLowerLimit;
  double kUpperLimit = ClimberConstants.kUpperLimit;
  double kMaxVelocity = ClimberConstants.kMaxVelocity;
  double kMaxAcceleration = ClimberConstants.kMaxAcceleration;

  public Climbers(int leftMotorId, int rightMotorId, int limitSwitchId) {
    m_leftClimber = new SparkMax(leftMotorId, MotorType.kBrushless);
    m_rightClimber = new SparkMax(rightMotorId, MotorType.kBrushless);
    m_leftClimber.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_rightClimber.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_leftController = m_leftClimber.getClosedLoopController();
    m_rightController = m_rightClimber.getClosedLoopController();
    m_leftEncoder = m_leftClimber.getEncoder();
    m_rightEncoder = m_rightClimber.getEncoder();
    limitSwitchClimber = new DigitalInput(0);
  }

  public static SparkMaxConfig leftConfig = new SparkMaxConfig();
  public static SparkMaxConfig rightConfig = new SparkMaxConfig();
  static {
      leftConfig.smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        .apply(new EncoderConfig().inverted(true))
        .apply(new ClosedLoopConfig().pid(ClimberConstants.kP, ClimberConstants.kI,
                                           ClimberConstants.kD)
                                     .apply(new MAXMotionConfig().positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                                                                 .maxAcceleration(0)
                                                                 .maxVelocity(0)));
      rightConfig.smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .apply(new EncoderConfig().inverted(false))
        .apply(new ClosedLoopConfig().pid(ClimberConstants.kP, ClimberConstants.kI,
                                          ClimberConstants.kD));
  }


  public double getPosition(){
    return m_leftEncoder.getPosition();
  }

  public boolean isNearPosition(double p){
    return MathUtil.isNear(p, getPosition(), ClimberConstants.kPositionTolerance);
  }

  public boolean limitSwitch(){
    return !limitSwitchClimber.get();
  }

  //todo should handle the limit switch in here somewhere
  public void setPosition(double position){
    position = MathUtil.clamp(position, kLowerLimit, kUpperLimit);
    if(limitSwitch()){
      position = Math.min(position, getPosition());
    }
    m_leftController.setReference(position, ControlType.kPosition);
    m_rightController.setReference(position, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void updateConfig(){
    SparkMaxConfig newConfig = new SparkMaxConfig();
    newConfig.apply(new ClosedLoopConfig()
                          .pid(kP, kI, kD).apply(new MAXMotionConfig()
                                                      .maxAcceleration(kMaxAcceleration)
                                                      .maxVelocity(kMaxVelocity)));
    m_leftClimber.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("kP", ()->kP, (x)->{kP = x; updateConfig(); });
    builder.addDoubleProperty("kI", ()->kI, (x)->{kI = x; updateConfig(); });
    builder.addDoubleProperty("kD", ()->kD, (x)->{kD = x; updateConfig(); });
    builder.addDoubleProperty("kMaxVelocity", ()->kMaxVelocity, (x)->{kMaxVelocity = x; updateConfig(); });
    builder.addDoubleProperty("kMaxAcceleration", ()->kMaxAcceleration, (x)->{kMaxAcceleration = x; updateConfig(); });
    builder.addDoubleProperty("kLowerLimit", ()->kLowerLimit, (x)->{kLowerLimit = x;});
    builder.addDoubleProperty("kUpperLimit", ()->kUpperLimit, (x)->{kUpperLimit = x;});
    builder.addBooleanProperty("limitSwitch", this::limitSwitch, null);
  }
}
