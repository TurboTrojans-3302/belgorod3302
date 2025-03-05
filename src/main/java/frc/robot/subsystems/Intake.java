// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private double inSpeed = Constants.IntakeConstants.inSpeed;
  private double outSpeed = Constants.IntakeConstants.outSpeed;
  private double upperLoadSpeed = Constants.IntakeConstants.upperLoadSpeed;

  private static final SparkMaxConfig lowerSparkConfig = new SparkMaxConfig();
  private static final SparkMaxConfig upperSparkConfig = new SparkMaxConfig();
  static {
    lowerSparkConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(40);
    upperSparkConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(40);
  }

  /** Creates a new Intake. */
  private SparkMax m_lowerIntakeMotor;
  private SparkMax m_upperIntakeMotor;
  private DigitalInput m_lowerObjectDetectSwitch;
  private DigitalInput m_upperObjectDetectSwitch;

  public Intake(int lowerSparkId, int upperSparkId, int lowerLimitSwitchId, int upperLimitSwitchId) {
    m_lowerIntakeMotor = new SparkMax(lowerSparkId, MotorType.kBrushless);
    m_upperIntakeMotor = new SparkMax(upperSparkId, MotorType.kBrushless);

    m_lowerObjectDetectSwitch = new DigitalInput(lowerLimitSwitchId);
    m_upperObjectDetectSwitch = new DigitalInput(upperLimitSwitchId);
    stop();
  }

  public void stop(){
    m_lowerIntakeMotor.set(0.0);
    m_upperIntakeMotor.set(0.0);
  }
  
  public double getLowerSpeed(){
    return m_lowerIntakeMotor.get();
  }

  /*
   * negative speed pulls in, positive speed pushes out
   */
  public void setLowerSpeed(double speed){
    m_lowerIntakeMotor.set(MathUtil.clamp(speed, Constants.IntakeConstants.intakeSpeedMin, Constants.IntakeConstants.intakeSpeedMax));
  }

  public boolean lowerObjectDetected(){
    return !m_lowerObjectDetectSwitch.get();
  }

  public double getUpperSpeed(){
    return m_upperIntakeMotor.get();
  }

  /*
   * negative speed moves down, positive speed moves up into gripper
   */
  public void setUpperSpeed(double speed){
    m_upperIntakeMotor.set(MathUtil.clamp(speed, Constants.IntakeConstants.intakeSpeedMin, Constants.IntakeConstants.intakeSpeedMax));
  }

  /*
   *  Object detected in the upper conveyor
   */
  public boolean upperObjectDetected(){
    return !m_lowerObjectDetectSwitch.get();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (lowerObjectDetected()){
      setLowerSpeed(Math.max(getLowerSpeed(), 0.0));
    }

  }

  public void in(){
    setLowerSpeed(inSpeed);
  }

  public void out(){
    setLowerSpeed(outSpeed);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("LowerSpeed", this::getLowerSpeed, this::setLowerSpeed);
    builder.addDoubleProperty("UpperSpeed", this::getUpperSpeed, this::setUpperSpeed);
    builder.addBooleanProperty("Lower Object Detected", this::lowerObjectDetected, null);
    builder.addBooleanProperty("Upper Object Detected", this::upperObjectDetected, null);
    builder.addDoubleProperty("inSpeed", ()->inSpeed, (x)->inSpeed = x);
    builder.addDoubleProperty("outSpeed", ()->outSpeed, (x)->outSpeed = x);
  }

  public void loadGripper() {
    setLowerSpeed(outSpeed);
    setUpperSpeed(upperLoadSpeed);
  }
}
