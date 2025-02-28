// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private double inSpeed = Constants.IntakeConstants.inSpeed;
  private double outSpeed = Constants.IntakeConstants.outSpeed;

  /** Creates a new Intake. */
  public SparkMax m_intake;
  public DigitalInput intakeLimitSwitch;

  public Intake(int motorId, int limitSwitchId) {
    m_intake = new SparkMax(motorId, MotorType.kBrushless);
    intakeLimitSwitch = new DigitalInput(limitSwitchId);
    m_intake.set(0.0);
  }
  
  public double getSpeed(){
    return m_intake.get();
  }

  /*
   * negative speed pulls in, positive speed pushes out
   */
  public void setSpeed(double speed){
    m_intake.set(MathUtil.clamp(speed, Constants.IntakeConstants.intakeSpeedMin, Constants.IntakeConstants.intakeSpeedMax));
  }

  public boolean objectDetected(){
    return !intakeLimitSwitch.get();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (objectDetected()){
      setSpeed(Math.max(getSpeed(), 0.0));
    }

  }

  public void in(){
    setSpeed(inSpeed);
  }

  public void out(){
    setSpeed(outSpeed);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Speed", this::getSpeed, this::setSpeed);
    builder.addBooleanProperty("Object Detected", this::objectDetected, null);
    builder.addDoubleProperty("inSpeed", ()->inSpeed, (x)->inSpeed = x);
    builder.addDoubleProperty("outSpeed", ()->outSpeed, (x)->outSpeed = x);
  }
}
