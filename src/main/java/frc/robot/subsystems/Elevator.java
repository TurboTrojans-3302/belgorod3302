// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//TODO No elevator object is actually created yet
//but the constants for it are
//TODO need to test what the high limit would be
package frc.robot.subsystems;

import java.util.Timer;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;



public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  public double elevatorSpeed;
  public SparkMax leftElevatorMotor;
  public SparkMax rightElevatorMotor;
  public DigitalInput elevatorHighLimitSwitch;
  public DigitalInput elevatorLowLimitSwitch;
  public boolean stopStatusHigh;
  public boolean stopStatusLow;
  public double elevatorPosition;
  public RelativeEncoder elevatorEncoder;
  
  


  public Elevator(int leftMotorID, int rightMotorId, int highSwitchId, int lowSwitchId) {
    leftElevatorMotor = new SparkMax(leftMotorID, MotorType.kBrushless);
    rightElevatorMotor = new SparkMax(leftMotorID, MotorType.kBrushless);
    elevatorHighLimitSwitch = new DigitalInput(highSwitchId);
    elevatorLowLimitSwitch = new DigitalInput(lowSwitchId);
    //true because elevator would start at lowest point
    stopStatusLow = true; 
    stopStatusHigh = false;
    elevatorSpeed = 0;
    leftElevatorMotor.set(elevatorSpeed);
    rightElevatorMotor.set(elevatorSpeed);
    //getEncoder() is a function already in the SparkBase class that creates a relative encoder if there isnt one
    //I would assume we only need one of the motors to use an encoder 
    elevatorEncoder = leftElevatorMotor.getEncoder();
    //Set position to starting position, where 0 equals the bottom of the elevator
    elevatorEncoder.setPosition(0);
  }

  
  public double getSpeed(){
    return elevatorSpeed;
  } 

  public double setSpeed(double speed){
    elevatorSpeed = MathUtil.clamp(speed, -Constants.ElevatorConstants.kElevatorMaxSpeed, Constants.ElevatorConstants.kElevatorMaxSpeed);
    leftElevatorMotor.set(elevatorSpeed);
    rightElevatorMotor.set(-elevatorSpeed);
    return elevatorSpeed;
  }

  public double getElevatorPosition(){
  //returns relative encoder position from one of the motors in full rotations
  //TODO test how many rotations get the elevator to its fullest extent from a starting position
   elevatorPosition = elevatorEncoder.getPosition();
   return elevatorPosition;

  }


  //stopping elevator with limit switches
  public void stopElevatorSwitch(){
    //see line 104 first
    //after checking for closed limit switches, the elevator is stopped if the planned speed would bring it towards the limit
    //if the elevator speed is not zero or going in the opposite direction of limit that has been triggered
//assuming negative speed is going down and vice versa
    if (stopStatusHigh){
      if ((!(elevatorSpeed == 0)) && (!(elevatorSpeed < 0))){
        leftElevatorMotor.set(0);
        rightElevatorMotor.set(0);
    }
  }
    if (stopStatusLow){
      if ((!(elevatorSpeed == 0)) && (!(elevatorSpeed > 0))){
        leftElevatorMotor.set(0);
        rightElevatorMotor.set(0);
      }
    }
   
  }
  //stopping without limit switches
  public double stopElevator(){
    elevatorSpeed = 0;
    return elevatorSpeed;
  }

  
  //digital input false means it is actually true because digital input switches return a zero when closed which corresponds to false
  public boolean limitCheck(){
    if (elevatorHighLimitSwitch.get() == false){
        stopStatusHigh = true;
        return stopStatusHigh;
    } else if (elevatorLowLimitSwitch.get() == false){
      stopStatusLow = true;
      return stopStatusLow;
    } else {
      return false;
    }
  }
//position in motor rotations
//sets automatic speed to either positive or negative based on where the elevator is in relation to target
//also slows down if it is within a range and stops when position is close to being reached
//input a position value from constants to get it to go to a certain level
  public double setPosition(double setPosition){
    if (setPosition > elevatorPosition){
      setSpeed(-Constants.ElevatorConstants.kElevatorAutoSpeed);
      
    } else if (setPosition < elevatorPosition){
      setSpeed(Constants.ElevatorConstants.kElevatorAutoSpeed);
      
    }  
    
    if ((setPosition == elevatorPosition + 5) || (setPosition == elevatorPosition - 5)){
      elevatorSpeed = elevatorSpeed * 0.75;
      return elevatorSpeed;
    } else if ((setPosition == elevatorPosition + 1) || (setPosition == elevatorPosition - 1)){
      elevatorSpeed = 0;
      return elevatorSpeed;
    }
        return elevatorSpeed;

  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //checks for limit switches
    limitCheck();
    //only runs if the limit check is true and the direction of planned travel is towards the limit switch as well.
    stopElevatorSwitch();

    

  }
}
