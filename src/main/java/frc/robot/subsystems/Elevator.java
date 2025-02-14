// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//TODO No elevator object is actually created yet
//but the constants for it are
//TODO need to test what the high limit would be
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  private double kMediumTolerance = 5;
  private double kSmallTolerance = 1;
  private double kElevatorMaxSpeed = ElevatorConstants.kElevatorMaxSpeed;

  public SparkMax leftElevatorMotor;
  public SparkMax rightElevatorMotor;
  public DigitalInput elevatorHighLimitSwitch;
  public DigitalInput elevatorLowLimitSwitch;
  public RelativeEncoder elevatorEncoder;

  public double kLevel1Trough = ElevatorConstants.kLevel1Trough;
  public double kLevel2 = ElevatorConstants.kLevel2;
  public double kLevel3 = ElevatorConstants.kLevel3;
  public double kLevel4 = ElevatorConstants.kLevel4;

  public Elevator(int leftMotorID, int rightMotorId, int highSwitchId, int lowSwitchId) {
    leftElevatorMotor = new SparkMax(leftMotorID, MotorType.kBrushless);
    rightElevatorMotor = new SparkMax(leftMotorID, MotorType.kBrushless);
    elevatorHighLimitSwitch = new DigitalInput(highSwitchId);
    elevatorLowLimitSwitch = new DigitalInput(lowSwitchId);
    // true because elevator would start at lowest point

    setMotorSpeed(0);

    // getEncoder() is a function already in the SparkBase class that creates a
    // relative encoder if there isnt one
    // I would assume we only need one of the motors to use an encoder
    elevatorEncoder = leftElevatorMotor.getEncoder();
    // Set position to starting position, where 0 equals the bottom of the elevator
    elevatorEncoder.setPosition(0);
  }

  public double getElevatorSpeed() {
    return elevatorEncoder.getVelocity();
  }

  public double setMotorSpeed(double speed) {
    double elevatorSpeed = MathUtil.clamp(speed, -kElevatorMaxSpeed, kElevatorMaxSpeed);
    leftElevatorMotor.set(elevatorSpeed);
    rightElevatorMotor.set(-elevatorSpeed);
    return elevatorSpeed;
  }

  public double getElevatorPosition() {
    // returns relative encoder position from one of the motors in full rotations
    // TODO test how many rotations get the elevator to its fullest extent from a
    // starting position

    return elevatorEncoder.getPosition();

  }

  // stopping elevator with limit switches
  public void checkLimits() {
    // see line 104 first
    // after checking for closed limit switches, the elevator is stopped if the
    // planned speed would bring it towards the limit
    // if the elevator speed is not zero or going in the opposite direction of limit
    // that has been triggered
    // assuming negative speed is going down and vice versa
    if (!elevatorHighLimitSwitch.get()) {
      if (getMotorSpeed() > 0.0) {
        setMotorSpeed(0.0);

      }
    }
    if (!elevatorLowLimitSwitch.get()) {
      if (getMotorSpeed() < 0.0) {
        setMotorSpeed(0.0);
      }
    }
  }

  private double getMotorSpeed() {
    return leftElevatorMotor.get();
  }

  // stopping without limit switches

  // digital input false means it is actually true because digital input switches
  // return a zero when closed which corresponds to false

  // position in motor rotations
  // sets automatic speed to either positive or negative based on where the
  // elevator is in relation to target
  // also slows down if it is within a range and stops when position is close to
  // being reached
  // input a position value from constants to get it to go to a certain level
  public void setPosition(double setPosition, double speed) {

    // you can change the speed reduction within tolerance here
    final double speedFactor = 0.75;
    double elevatorPosition = getElevatorPosition();

    if (setPosition > (elevatorPosition + kMediumTolerance)) {
      setMotorSpeed(-speed);

    } else if (setPosition > (elevatorPosition + kSmallTolerance)) {
      setMotorSpeed(-speed * speedFactor);

    } else if (setPosition > (elevatorPosition - kSmallTolerance)) {
      setMotorSpeed(0.0);

    } else if (setPosition > (elevatorPosition - kMediumTolerance)) {
      setMotorSpeed(speed * speedFactor);

    } else {
      setMotorSpeed(speed);
    }
  }

  public boolean isNear(double position) {
    return MathUtil.isNear(position, getElevatorSpeed(), kSmallTolerance);
  }

  public boolean isNearLevel1(){ return isNear(kLevel1Trough); }
  public boolean isNearLevel2(){ return isNear(kLevel2); }
  public boolean isNearLevel3(){ return isNear(kLevel3); }
  public boolean isNearLevel4(){ return isNear(kLevel4); }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // checks for limit switches

    // only runs if the limit check is true and the direction of planned travel is
    // towards the limit switch as well.
    checkLimits();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Position", this::getElevatorPosition, null);
    builder.addDoubleProperty("MaxSpeed", () -> kElevatorMaxSpeed, (x) -> { kElevatorMaxSpeed = x; });
    builder.addDoubleProperty("Tolerance", ()-> kSmallTolerance, (x)->{ kSmallTolerance=x; });
    builder.addDoubleProperty("Level1Trough", () -> kLevel1Trough, (x) -> { kLevel1Trough = x; });
    builder.addDoubleProperty("Level2", () -> kLevel2, (x) -> { kLevel2 = x; });
    builder.addDoubleProperty("Level3", () -> kLevel3, (x) -> { kLevel3 = x; });
    builder.addDoubleProperty("Level4", () -> kLevel4, (x) -> { kLevel4 = x; });
    builder.addBooleanProperty("AtLevel1", this::isNearLevel1, null );
    builder.addBooleanProperty("AtLevel1", this::isNearLevel2, null );
    builder.addBooleanProperty("AtLevel1", this::isNearLevel3, null );
    builder.addBooleanProperty("AtLevel1", this::isNearLevel4, null );
  }
}
