// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Gripper extends SubsystemBase {
  /** Creates a new Gripper. */
  SparkMax gripperMotor;
  SparkMax gripperExtension;
  DigitalInput gripperClosedSwitch;
  DigitalInput gripperFullyRetracted;
  RelativeEncoder extensionMotorEncoder;
  double gripperSpeed = Constants.GripperConstants.gripperMotorSpeed;
  double extensionSpeed = Constants.GripperConstants.gripperExtensionSpeed;
  final double extensionToleranceMedium = 4.0;
  final double extensionToleranceSmall = 2.0;
  final double stopTolerance = 0.5;


  public Gripper(int gripperMotorID, int gripperExtensionID, int closedSwitchID, int retractedSwitchID) {
    gripperMotor = new SparkMax(gripperMotorID, MotorType.kBrushless);
    gripperExtension = new SparkMax(gripperExtensionID, MotorType.kBrushless);
    extensionMotorEncoder = gripperExtension.getEncoder();
    gripperClosedSwitch = new DigitalInput(closedSwitchID);
    gripperFullyRetracted = new DigitalInput(retractedSwitchID);
  }

  public boolean gripperClosed(){
    return !gripperClosedSwitch.get();
  }

  public boolean isExtensionRetracted(){
    return !gripperFullyRetracted.get();
  }

  public void closeGripper(){
    if (gripperClosed()){
      //use less pressure when holding the object but continue to hold
      gripperMotor.set(gripperSpeed * 0.5);
    } else {
      gripperMotor.set(gripperSpeed);
    }
    

  }

  public void openGripper(){
    gripperMotor.set(-gripperSpeed);
    new WaitCommand(0.5);
    gripperMotor.set(0.0);
  }

  public void extendGripper(){
    if (getExtensionPosition() > Constants.GripperConstants.gripperExtendedPosition - extensionToleranceMedium){
      gripperExtension.set(extensionSpeed * 0.75);
    } else if (getExtensionPosition() > Constants.GripperConstants.gripperExtendedPosition - extensionToleranceSmall){
      gripperExtension.set(extensionSpeed * 0.25);
    } else if (getExtensionPosition() > Constants.GripperConstants.gripperExtendedPosition - stopTolerance){
      gripperExtension.set(0);

    }
    
    


  }

  public void retractGripper(){
    if (isExtensionRetracted()){
      gripperExtension.set(0);
    } else {
      gripperExtension.set(-extensionSpeed);
    }
  }

  public boolean isGripperFullyExtended(){
    return (getExtensionPosition() > Constants.GripperConstants.gripperExtendedPosition - stopTolerance);
  }
  public double getExtensionPosition(){
    return extensionMotorEncoder.getPosition();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    gripperClosed();
  }
}
