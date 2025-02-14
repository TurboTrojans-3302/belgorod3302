// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  SparkMax armRotator;
  RelativeEncoder armEncoder;
  DigitalInput limitSwitchElevator;
  Boolean atElevator;
  //Tolerances for elevator position in relation to destination, causes the arm speed to slow down or stop.
  public static final double kSmallTolerance = 2.0;
  public static final double kMediumTolerance = 7.0;

  public Arm(int motorID, int limitSwitchID) {

    armRotator = new SparkMax(motorID, MotorType.kBrushless);
    armEncoder = armRotator.getEncoder();
    armEncoder.setPosition(0.0);
    atElevator = false;

  }


  public double getArmSpeed(){
    return armEncoder.getVelocity();
  }

  public double getMotorSpeed(){
    return armRotator.get();
  }

  public double getArmPosition(){
    return armEncoder.getPosition();
  }

  public double setArmSpeed(double speed){
    double armSpeed = MathUtil.clamp(speed, -Constants.ArmConstants.armMaxSpeed, Constants.ArmConstants.armMaxSpeed);
    armRotator.set(armSpeed);
    return armSpeed;
  }

  public boolean AtElevator(){
    return !limitSwitchElevator.get();
  }

  public void moveToElevator(double speed){
    //increasing motor position indicates movement towards the elevator hopefully
    //probably need actual PID
    double armSpeed = speed;
    if (getArmPosition() > Constants.ArmConstants.armPositionElevator - kMediumTolerance){
      setArmSpeed(armSpeed * 0.75); 

    } else if (getArmPosition() > Constants.ArmConstants.armPositionElevator - kSmallTolerance) {
      setArmSpeed(armSpeed * 0.25);

    } else if (AtElevator()){
      setArmSpeed(0);

    } else { 
      //hasnt reached a tolerance yet
      setArmSpeed(armSpeed);
    }

   
    
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    AtElevator();
  }
}
