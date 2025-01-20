// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;

public class Climbers extends SubsystemBase {
  /** Creates a new Climbers. */
  SparkMax m_rightClimber;
  SparkMax m_leftClimber;
  SparkMaxConfig leftConfig;
  //used for detecting when climber is at highest position
  DigitalInput limitSwitchClimber;
  double climberSpeed;
  boolean maxHeight;
  ResetMode leftResetMode;
  PersistMode leftPersistMode;

  public Climbers(int leftMotorId, int rightMotorId, int limitSwitchId) {
    m_leftClimber = new SparkMax(leftMotorId, MotorType.kBrushless);
    m_rightClimber = new SparkMax(rightMotorId, MotorType.kBrushless);
    leftConfig = new SparkMaxConfig(); 
    leftConfig.inverted(true);
    //applies the inverted configuration without resetting all parameters and making sure they arent permanent with the no persist part.
    m_leftClimber.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    limitSwitchClimber = new DigitalInput(0);
    climberSpeed = 0;
    maxHeight = false;
  }

  public double getSpeed(){
    return climberSpeed;
  }
  
  public boolean limitCheck(){
    if (!limitSwitchClimber.get()){
      maxHeight = true;
      return maxHeight;
    } else {
      maxHeight = false;
      return maxHeight;
    }
  }
  public double setSpeed(double speed) {
    if (maxHeight && climberSpeed > 0){
      climberSpeed = 0;
      m_leftClimber.set(climberSpeed);
      m_rightClimber.set(climberSpeed);
      return climberSpeed;
    } else {
      climberSpeed = MathUtil.clamp(speed, -Constants.ClimberConstants.climberMaxSpeed, Constants.ClimberConstants.climberMaxSpeed);
      m_leftClimber.set(climberSpeed);
      m_rightClimber.set(climberSpeed);
      return climberSpeed;
    }
  }

  public double ClimbersUp(){
    if (maxHeight){
      climberSpeed = 0;
      m_leftClimber.set(climberSpeed);
      m_rightClimber.set(climberSpeed);
      return climberSpeed;
    } else {
      climberSpeed = Constants.ClimberConstants.climberAutoSpeed;
      m_leftClimber.set(climberSpeed);
      m_rightClimber.set(climberSpeed);
      return climberSpeed;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
