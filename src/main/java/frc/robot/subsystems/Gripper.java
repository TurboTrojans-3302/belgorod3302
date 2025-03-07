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
import frc.robot.Constants.GripperConstants;

public class Gripper extends SubsystemBase {
  /** Creates a new Gripper. */
  SparkMax gripperMotor;
  SparkMax gripperExtension;
  DigitalInput gripperClosedSwitch;
  DigitalInput gripperFullyRetracted;
  DigitalInput gripperObjectDetected;
  RelativeEncoder extensionMotorEncoder;
  RelativeEncoder gripperEncoder;
  double extensionSpeed = GripperConstants.gripperExtensionSpeed;
  final double extensionToleranceMedium = 4.0;
  final double extensionToleranceSmall = 2.0;
  final double stopTolerance = 0.5;
  PIDController gripperPID;
  double kP = GripperConstants.kP;
  double kI = GripperConstants.kI;
  double kD = GripperConstants.kD;
  double kClosed = GripperConstants.closedPosition;
  double kOpen = GripperConstants.openPosition;
  double kExtendedPos = GripperConstants.gripperExtendedPosition;
  double kRetractedPos = GripperConstants.gripperRetractedPosition;

  public Gripper(int gripperMotorID, int gripperExtensionID, int closedSwitchID, int retractedSwitchID,
      int objectDetectionID) {
    gripperMotor = new SparkMax(gripperMotorID, MotorType.kBrushless);
    gripperExtension = new SparkMax(gripperExtensionID, MotorType.kBrushless);
    extensionMotorEncoder = gripperExtension.getEncoder();
    gripperEncoder = gripperMotor.getEncoder();
    gripperClosedSwitch = new DigitalInput(closedSwitchID);
    gripperFullyRetracted = new DigitalInput(retractedSwitchID);
    gripperObjectDetected = new DigitalInput(objectDetectionID);
    gripperPID = new PIDController(kP, kI, kD);
  }

  public double getGripperPosition() {
    return gripperEncoder.getPosition();
  }

  public boolean isGripperClosed() {
    return getGripperPosition() <= kClosed;
  }

  public boolean isGripperOpen() {
    return getGripperPosition() >= kOpen;
  }

  public boolean isExtensionRetracted() {
    return !gripperFullyRetracted.get();
  }

  public void closeGripper() {
    gripperPID.setSetpoint(kClosed);
  }

  public void openGripper() {
    gripperPID.setSetpoint(kOpen);
  }

  public void toggleGripper() {
    if (isGripperOpen()) {
      closeGripper();
    } else {
      openGripper();
    }
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

  public boolean isGripperFullyExtended() {
    return (getExtensionPosition() > kExtendedPos - stopTolerance);
  }

  public void setGripperPosition(double setpoint) {
    gripperPID.setSetpoint(setpoint);
  }

  public double getExtensionPosition() {
    return extensionMotorEncoder.getPosition();
  }

  public boolean objectInGripper() {
    return (!gripperObjectDetected.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double position = gripperEncoder.getPosition();
    double speed = gripperPID.calculate(position);
    gripperMotor.set(speed);
  }

  public Command extendCommand() {
    return new FunctionalCommand(() -> extendGripper(),
        () -> {
        },
        (x) -> {
        },
        () -> isGripperFullyExtended(),
        this
    );
  }

  public Command retractCommand() {
    return new FunctionalCommand(() -> retractGripper(),
        () -> {
        },
        (x) -> {
        },
        () -> isExtensionRetracted(),
        this
    );
  }

  public Command openCommand() {
    return new FunctionalCommand(() -> openGripper(),
        () -> {
        },
        (x) -> {
        },
        () -> isGripperOpen(),
        this
    );
  }

  public Command closeCommand() {
    return new FunctionalCommand(() -> closeGripper(),
        () -> {
        },
        (x) -> {
        },
        () -> isGripperClosed(),
        this
    );
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("gripperP", () -> kP, (x) -> {
      kP = x;
    });
    builder.addDoubleProperty("gripperI", () -> kI, (x) -> {
      kI = x;
    });
    builder.addDoubleProperty("gripperD", () -> kD, (x) -> {
      kD = x;
    });
    builder.addDoubleProperty("kClosed", () -> kClosed, (x) -> {
      kClosed = x;
    });
    builder.addDoubleProperty("kOpen", () -> kOpen, (x) -> {
      kOpen = x;
    });
    builder.addBooleanProperty("Obj In Gripper", this::objectInGripper, null);
    builder.addBooleanProperty("Open", this::isGripperOpen, null);
    builder.addBooleanProperty("Closed", this::isGripperClosed, null);

    
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
