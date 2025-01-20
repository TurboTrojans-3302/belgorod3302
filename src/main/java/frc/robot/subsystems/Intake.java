// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public SparkMax m_intake;
  public double intakeSpeed;
  public DigitalInput intakeLimitSwitch;
  public boolean objectDetected;



  public Intake(int motorId, int limitSwitchId) {
    m_intake = new SparkMax(motorId, MotorType.kBrushless);
    intakeLimitSwitch = new DigitalInput(limitSwitchId);
    intakeSpeed = 0;
    m_intake.set(intakeSpeed);
    objectDetected = false;
  }
  
  public double getSpeed(){
    return intakeSpeed;

  }
  public double setSpeed(double speed){
    if (objectDetected){
      if (getSpeed() > 0){
        intakeSpeed = 0;
        return intakeSpeed;
      } else {
        intakeSpeed = MathUtil.clamp(speed, Constants.IntakeConstants.intakeSpeedMin, Constants.IntakeConstants.intakeSpeedMax);
        m_intake.set(intakeSpeed);
        return intakeSpeed;
      }
      
    } 
    return intakeSpeed;
    

  }

  public boolean limitSwitch(){
    if (!intakeLimitSwitch.get()){
      objectDetected = true;
      m_intake.set(0);
      return objectDetected;
    } else {
      objectDetected = false;
      return objectDetected;
    }

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    limitSwitch();
    getSpeed();
  }
}
