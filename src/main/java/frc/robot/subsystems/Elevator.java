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

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  private double kMediumTolerance = 5;
  private double kSmallTolerance = 1;
  private double kElevatorMaxSpeed = ElevatorConstants.kElevatorMaxSpeed;

  public SparkMax leftElevatorMotor;
  public SparkMax rightElevatorMotor;
  public PIDController elevatorPID; //todo make this a ProfiledPIDController
  double kP = Constants.ElevatorConstants.kP;
  double kI = Constants.ElevatorConstants.kI;
  double kD = Constants.ElevatorConstants.kD;
  public DigitalInput elevatorHighLimitSwitch;
  public DigitalInput elevatorLowLimitSwitch;
  public RelativeEncoder elevatorEncoder;

  public double kLoadPosition = ElevatorConstants.kLoadPosition;
  public double kLevel1Trough = ElevatorConstants.kLevel1Trough;
  public double kLevel2 = ElevatorConstants.kLevel2;
  public double kLevel3 = ElevatorConstants.kLevel3;
  public double kLevel4 = ElevatorConstants.kLevel4;

  public Elevator(int leftMotorID, int rightMotorId, int highSwitchId, int lowSwitchId) {
    leftElevatorMotor = new SparkMax(leftMotorID, MotorType.kBrushless);
    rightElevatorMotor = new SparkMax(leftMotorID, MotorType.kBrushless);
    elevatorHighLimitSwitch = new DigitalInput(highSwitchId);
    elevatorLowLimitSwitch = new DigitalInput(lowSwitchId);
    elevatorPID = new PIDController(kP, kI, kD);
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

  public void changeSetPoint(double delta){
    double p = getElevatorPosition();
    setPositionPID(p + delta);
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

  public void setPositionPID(double setpoint){
     elevatorPID.setSetpoint(setpoint);
  }

  public Command setPostionCommand(double setpoint){
    return new FunctionalCommand(()->setPositionPID(setpoint),
                                 null,
                                 null,
                                 ()->atSetpoint()
                                );
  }

  public Command loadPosCommand() { return setPostionCommand(kLoadPosition); }
  public Command level1Command() { return setPostionCommand(kLevel1Trough); }
  public Command level2Command() { return setPostionCommand(kLevel2); }
  public Command level3Command() { return setPostionCommand(kLevel3); }
  public Command level4Command() { return setPostionCommand(kLevel4); }

  public boolean atSetpoint(){
    return elevatorPID.atSetpoint();
  }

   public boolean isNear(double position) {
    return MathUtil.isNear(position, getElevatorSpeed(), kSmallTolerance);
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // checks for limit switches

    // only runs if the limit check is true and the direction of planned travel is
    // towards the limit switch as well.
    checkLimits();
    double speedCalculation = elevatorPID.calculate(getElevatorPosition());
     leftElevatorMotor.set(speedCalculation);
     rightElevatorMotor.set(-speedCalculation);
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
    builder.addBooleanProperty("AtLevel1", ()->isNear(kLevel1Trough), null );
    builder.addBooleanProperty("AtLevel2", ()->isNear(kLevel2), null );
    builder.addBooleanProperty("AtLevel3", ()->isNear(kLevel3), null );
    builder.addBooleanProperty("AtLevel4", ()->isNear(kLevel4), null );
    builder.addDoubleProperty("elevator kP", ()-> kP, (x) -> {kP = x;});
    builder.addDoubleProperty("elevator kI", ()-> kI, (x) -> {kI = x;});
    builder.addDoubleProperty("elevator kD", ()-> kD, (x) -> {kD = x;});
  }
}
