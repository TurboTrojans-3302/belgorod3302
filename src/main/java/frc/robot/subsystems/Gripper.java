// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase {
  /** Creates a new Gripper. */
  SparkMax gripperExtension;
  DigitalInput gripperClosedSwitch;
  DigitalInput gripperFullyRetracted;
  RelativeEncoder extensionMotorEncoder;
  double extensionSpeed = GripperConstants.gripperExtensionSpeed;
  final double extensionToleranceMedium = 4.0;
  final double extensionToleranceSmall = 2.0;
  final double stopTolerance = 0.5;
  double kExtendedPos = GripperConstants.gripperExtendedPosition;
  double kRetractedPos = GripperConstants.gripperRetractedPosition;

  public Gripper(int gripperMotorID, int gripperExtensionID, int closedSwitchID, int retractedSwitchID,
      int objectDetectionID) {
    gripperExtension = new SparkMax(gripperExtensionID, MotorType.kBrushless);
    gripperExtension.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    extensionMotorEncoder = gripperExtension.getEncoder();
    gripperClosedSwitch = new DigitalInput(closedSwitchID);
    gripperFullyRetracted = new DigitalInput(retractedSwitchID);
  }

  public boolean isExtensionRetracted() {
    return !gripperFullyRetracted.get();
  }

  public void extendGripper() {
    if (getExtensionPosition() > kExtendedPos - extensionToleranceMedium) {
      gripperExtension.set(extensionSpeed * 0.75);
    } else if (getExtensionPosition() > kExtendedPos - extensionToleranceSmall) {
      gripperExtension.set(extensionSpeed * 0.25);
    } else if (getExtensionPosition() > kExtendedPos - stopTolerance) {
      gripperExtension.set(0);

    }
  }

  public void retractGripper() {
    if (isExtensionRetracted()) {
      gripperExtension.set(0);
    } else {
      gripperExtension.set(-extensionSpeed);
    }
  }

  public void stopExtention(){
    gripperExtension.set(0);
  }

  public boolean isGripperFullyExtended() {
    return (getExtensionPosition() > kExtendedPos - stopTolerance);
  }

  public double getExtensionPosition() {
    return extensionMotorEncoder.getPosition();
  }

  public Command extendCommand() {
    return new FunctionalCommand(
        ()->{},
        this::extendGripper,
        (x) -> this.stopExtention(),
        this::isGripperFullyExtended,
        this
    );
  }

  public Command retractCommand() {
    return new FunctionalCommand(
      ()->{},
      this::retractGripper,
      (x) -> this.stopExtention(),
      this::isExtensionRetracted,
      this
    );
  }


  public Command testExtensionCommand(double speed) {
    return new FunctionalCommand(
        () -> gripperExtension.set(speed),
        () -> {},
        (x) -> gripperExtension.set(0.0),
        () -> false,
        this
    );
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    
    builder.addDoubleProperty("Extension Pos.", this::getExtensionPosition, null);
    builder.addBooleanProperty("Extended", this::isGripperFullyExtended, null);
    builder.addBooleanProperty("Retracted", this::isExtensionRetracted, null);
    builder.addDoubleProperty("kExtendedPos", () -> kExtendedPos, (x) -> {
      kExtendedPos = x;
    });
    builder.addDoubleProperty("kRetractedPos", () -> kRetractedPos, (x) -> {
      kRetractedPos = x;
    });
  }
}
