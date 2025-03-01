// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gripper extends SubsystemBase {
  /** Creates a new Gripper. */
  SparkMax gripperMotor;
  SparkMax gripperExtension;
  DigitalInput gripperClosedSwitch;
  DigitalInput gripperFullyRetracted;
  DigitalInput gripperObjectDetected;
  RelativeEncoder extensionMotorEncoder;
  RelativeEncoder gripperEncoder;
  double gripperSpeed = Constants.GripperConstants.gripperMotorSpeed;
  double extensionSpeed = Constants.GripperConstants.gripperExtensionSpeed;
  final double extensionToleranceMedium = 4.0;
  final double extensionToleranceSmall = 2.0;
  final double stopTolerance = 0.5;
  PIDController gripperPID;
  double kP = Constants.GripperConstants.kP;
  double kI = Constants.GripperConstants.kI;
  double kD = Constants.GripperConstants.kD;

  
  public Gripper(int gripperMotorID, int gripperExtensionID, int closedSwitchID, int retractedSwitchID, int objectDetectionID) {
    gripperMotor = new SparkMax(gripperMotorID, MotorType.kBrushless);
    gripperExtension = new SparkMax(gripperExtensionID, MotorType.kBrushless);
    extensionMotorEncoder = gripperExtension.getEncoder();
    gripperEncoder = gripperMotor.getEncoder();
    gripperClosedSwitch = new DigitalInput(closedSwitchID);
    gripperFullyRetracted = new DigitalInput(retractedSwitchID);
    gripperObjectDetected = new DigitalInput(objectDetectionID);
    gripperPID = new PIDController(kP, kI, kD);
  }

  public double getGripperPosition(){
    return gripperEncoder.getPosition();
  }
  public boolean isGripperClosed(){
    return getGripperPosition() <= Constants.GripperConstants.closedPosition;
  }

  public boolean isGripperOpen(){
    return getGripperPosition() >= Constants.GripperConstants.openPosition;
  }

  public boolean isExtensionRetracted(){
    return !gripperFullyRetracted.get();
  }

  public void closeGripper(){
    gripperPID.setSetpoint(Constants.GripperConstants.closedPosition);
  

  }

  public void openGripper(){
    gripperPID.setSetpoint(Constants.GripperConstants.openPosition);
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
  
  public void setGripperPosition(double setpoint){
    gripperPID.setSetpoint(setpoint);
  }

  public double getExtensionPosition(){
    return extensionMotorEncoder.getPosition();
  }

  public boolean objectInGripper(){
    return (!gripperObjectDetected.get());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double position = gripperEncoder.getPosition();
    double speed = gripperPID.calculate(position);
    gripperMotor.set(speed);
  }

  public Command extendCommand(){
    return new FunctionalCommand( ()-> extendGripper(),
                                  null,
                                  null,
                                  ()-> isGripperFullyExtended()
                                );
  }

  public Command retractCommand(){
    return new FunctionalCommand( ()-> retractGripper(),
                                  null,
                                  null,
                                  ()-> isExtensionRetracted()
                                );
  }

  public Command openCommand(){
    return new FunctionalCommand( ()-> openGripper(),
                                  null,
                                  null,
                                  ()-> isGripperOpen()
                                );
  }

  public Command closeCommand(){
    return new FunctionalCommand( ()-> closeGripper(),
                                  null,
                                  null,
                                  ()-> isGripperClosed()
                                );
  }

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("gripperP", ()-> kP, (x)->{ kP = x;});
    builder.addDoubleProperty("gripperI", ()-> kI, (x)->{ kI = x;});
    builder.addDoubleProperty("gripperD", ()-> kD, (x)->{ kD = x;});
  }
}
