// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
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
  double kUpperLimitRight = ClimberConstants.kUpperLimitRight;
  double kUpperLimitLeft = ClimberConstants.kUpperLimitLeft;
  double lockedPosition = ClimberConstants.kLockedPosition;
  double kMaxVelocity = ClimberConstants.kMaxVelocity;
  double kMaxAcceleration = ClimberConstants.kMaxAcceleration;
  public double testSpeed = 0;

  double positionSetPoint = 0.0;
  double increment = ClimberConstants.increment;
  public boolean climberLockActive = false;

  public Climbers(int leftMotorId, int rightMotorId) {
    m_leftClimber = new SparkMax(leftMotorId, MotorType.kBrushless);
    m_rightClimber = new SparkMax(rightMotorId, MotorType.kBrushless);
    m_leftClimber.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_rightClimber.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_leftController = m_leftClimber.getClosedLoopController();
    m_rightController = m_rightClimber.getClosedLoopController();
    m_leftEncoder = m_leftClimber.getEncoder();
    m_rightEncoder = m_rightClimber.getEncoder();
   

    m_leftEncoder.setPosition(ClimberConstants.kUpperLimitLeft);
    m_rightEncoder.setPosition(ClimberConstants.kUpperLimitRight);
    positionSetPoint = 0.0;
  }

  public static SparkMaxConfig leftConfig = new SparkMaxConfig();
  public static SparkMaxConfig rightConfig = new SparkMaxConfig();
  static {
    // positive motor speed is lifting us up
      leftConfig.smartCurrentLimit(40)
        .idleMode(IdleMode.kCoast)
        .inverted(true)
        .apply(new ClosedLoopConfig().pid(ClimberConstants.kP, ClimberConstants.kI,
                                           ClimberConstants.kD)
                                      .apply(new MAXMotionConfig().positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                                                                  .maxAcceleration(ClimberConstants.kMaxAcceleration)
                                                                  .maxVelocity(ClimberConstants.kMaxVelocity)));
      rightConfig.smartCurrentLimit(40)
        .idleMode(IdleMode.kCoast)
        .inverted(false)
        .apply(new ClosedLoopConfig().pid(ClimberConstants.kP, ClimberConstants.kI,
                                          ClimberConstants.kD)
                                    .apply(new MAXMotionConfig().positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                                                                .maxAcceleration(ClimberConstants.kMaxAcceleration)
                                                                .maxVelocity(ClimberConstants.kMaxVelocity)));

;
  }

  public void test(double speed){
    m_leftClimber.set(speed);
    m_rightClimber.set(speed);
  }

  public double getPositionLeft(){
    return m_leftEncoder.getPosition();
  }

  public double getPositionRight(){
    return m_rightEncoder.getPosition();
  }

 

  

  public void setPosition(double position){
    setPosition(position, false);
  }
  
  public void setPosition(double position, boolean override){
    //TODO don't move if we're in the lock position)

    double Lposition, Rposition;
    if (override == false){
      Rposition = MathUtil.clamp(position, kLowerLimit, kUpperLimitRight);
      Lposition = MathUtil.clamp(position, kLowerLimit, kUpperLimitLeft);
      
    }else{
      Rposition = position;
      Lposition = position;
    }

    
    m_leftController.setReference(Lposition, ControlType.kPosition);
    m_rightController.setReference(Rposition, ControlType.kPosition);
    positionSetPoint = position;
  }

  public double getSetpoint(){
    return positionSetPoint;
  }

  public Command climbersUpCommand(){
    return new FunctionalCommand(()->{},
                                 ()->setPosition(getPositionLeft() + increment),
                                 (x)->{},
                                 ()->false,
                                 this);
  }

  public Command climbersDownCommand(){
    return new FunctionalCommand(()->{},
                                 ()->setPosition(getPositionLeft() - increment),
                                 (x)->{},
                                 ()->false,
                                 this);
  }

  public void climbersFullDown(){
    if (climberLockActive){
      setPosition(lockedPosition, true);
    }
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
    m_rightClimber.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("increment", ()->this.increment, (x)->increment=x);
    builder.addDoubleProperty("Setpoint", this::getSetpoint, this::setPosition);
    builder.addDoubleProperty("Right Pos", this::getPositionRight, null);
    builder.addDoubleProperty("Left Pos", this::getPositionLeft, null);
    builder.addDoubleProperty("kP", ()->kP, (x)->{kP = x; updateConfig(); });
    builder.addDoubleProperty("kI", ()->kI, (x)->{kI = x; updateConfig(); });
    builder.addDoubleProperty("kD", ()->kD, (x)->{kD = x; updateConfig(); });
    builder.addDoubleProperty("kMaxVelocity", ()->kMaxVelocity, (x)->{kMaxVelocity = x; updateConfig(); });
    builder.addDoubleProperty("kMaxAcceleration", ()->kMaxAcceleration, (x)->{kMaxAcceleration = x; updateConfig(); });
    builder.addDoubleProperty("kLowerLimit", ()->kLowerLimit, (x)->{kLowerLimit = x;});
    builder.addDoubleProperty("kUpperLimitLeft", ()->kUpperLimitLeft, (x)->{kUpperLimitLeft = x;});
    builder.addDoubleProperty("kUpperLimitRight", ()->kUpperLimitRight, (x)->{kUpperLimitRight = x;});
    builder.addDoubleProperty("lockedPosition", ()->lockedPosition, (x)->{lockedPosition = x;});
    builder.addDoubleProperty("testSpeed", ()->testSpeed, (x)->{testSpeed = x;});
    builder.addBooleanProperty("LockEnable", ()->climberLockActive, null);
    
    builder.addDoubleProperty("leftRPM", m_leftEncoder::getVelocity, null);
    builder.addDoubleProperty("leftOutput", m_leftClimber::getAppliedOutput, null);
   
  }
}
