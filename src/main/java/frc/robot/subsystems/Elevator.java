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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  private double kTolerance = ElevatorConstants.kTolerance;
  private double kElevatorMaxSpeed = ElevatorConstants.kElevatorMaxSpeed;

  public SparkMax leftElevatorMotor;
  public SparkMax rightElevatorMotor;
  public ProfiledPIDController elevatorPID;
  double kP = Constants.ElevatorConstants.kP;
  double kI = Constants.ElevatorConstants.kI;
  double kD = Constants.ElevatorConstants.kD;
  public DigitalInput elevatorHighLimitSwitch;
  public DigitalInput elevatorLowLimitSwitch;
  public RelativeEncoder elevatorEncoder;

  public double kLimitLow = ElevatorConstants.kLimitLow;
  public double kLimitHigh = ElevatorConstants.kLimitHigh;
  public double kSoftLimitLow = ElevatorConstants.kSoftLimitLow;
  public double kSoftLimitHigh = ElevatorConstants.kSoftLimitHigh;
  public double kLoadPosition = ElevatorConstants.kLoadPosition;
  public double kLevel1Trough = ElevatorConstants.kLevel1Trough;
  public double kPickupLevel = ElevatorConstants.kPickupLevel;
  public double kLevel2 = ElevatorConstants.kLevel2;
  public double kLevel3 = ElevatorConstants.kLevel3;
  public double kLevel4 = ElevatorConstants.kLevel4;
  private double kElevatorMaxAccel = ElevatorConstants.kElevatorMaxAccel;
  
    public Elevator(int leftMotorID, int rightMotorId, int highSwitchId, int lowSwitchId) {
      leftElevatorMotor = new SparkMax(leftMotorID, MotorType.kBrushless);
      rightElevatorMotor = new SparkMax(leftMotorID, MotorType.kBrushless);
      elevatorHighLimitSwitch = new DigitalInput(highSwitchId);
      elevatorLowLimitSwitch = new DigitalInput(lowSwitchId);
      updateCfg();

      // true because elevator would start at lowest point
  
      stop();
  
      // getEncoder() is a function already in the SparkBase class that creates a
      // relative encoder if there isnt one
      // I would assume we only need one of the motors to use an encoder
      elevatorEncoder = leftElevatorMotor.getEncoder();
    }
  
    private void updateCfg(){
      elevatorPID = new ProfiledPIDController(kP, kI, kD, new Constraints(kElevatorMaxSpeed, kElevatorMaxAccel));
      elevatorPID.setTolerance(kTolerance);
      elevatorPID.reset(getElevatorPosition());
    }

    public double getElevatorSpeed() {
      return elevatorEncoder.getVelocity();
    }
  
    private double setMotorSpeed(double speed) {
      leftElevatorMotor.set(speed);
      rightElevatorMotor.set(-speed);
      return speed;
    }
  
    public double getElevatorPosition() {
      // returns relative encoder position from one of the motors in full rotations
      // TODO test how many rotations get the elevator to its fullest extent from a
      // starting position
  
      return elevatorEncoder.getPosition();
    }
  
    public void resetElevatorPositionEncoder(double position) {
      elevatorEncoder.setPosition(position);
      updateCfg();
    }
  
    // stopping elevator with limit switches
    public void checkLimits() {
      /*
       * Checking the hard limits
       */
      if (!elevatorHighLimitSwitch.get()) {
        if (getElevatorPosition() < kLimitHigh) {
          resetElevatorPositionEncoder(kLimitHigh);
        }
      }
      if (!elevatorLowLimitSwitch.get()) {
        if (getElevatorPosition() > kLimitLow) {
          resetElevatorPositionEncoder(kLimitLow);
        }
      }
  
      /*
       * Checking the soft limits
       */
      if (getElevatorPosition() > kSoftLimitHigh) {
        if (getMotorSpeed() > 0.0) {
          setMotorSpeed(0.0);
        }
      }
      if (getElevatorPosition() < kSoftLimitLow) {
        if (getMotorSpeed() < 0.0) {
          setMotorSpeed(0.0);
        }
      }
    }
  
    private double getMotorSpeed() {
      return leftElevatorMotor.get();
    }
  
    public void setPosition(double setpoint) {
      setpoint = MathUtil.clamp(setpoint, kSoftLimitLow, kSoftLimitHigh);
      elevatorPID.setGoal(setpoint);
    }
  
    public boolean isNear(double position) {
      return MathUtil.isNear(position, getElevatorSpeed(), kTolerance);
    }
  
    public boolean isNearPickupLevel() {
      return isNear(kPickupLevel);
    }
  
    public boolean isNearLevel1() {
      return isNear(kLevel1Trough);
    }
  
    public boolean isNearLevel2() {
      return isNear(kLevel2);
    }
  
    public boolean isNearLevel3() {
      return isNear(kLevel3);
    }
  
    public boolean isNearLevel4() {
      return isNear(kLevel4);
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      // checks for limit switches
  
      // todo I bet we need some feedforward here
      double speedCalculation = elevatorPID.calculate(getElevatorPosition());
      setMotorSpeed(speedCalculation);
  
      // important to check the limits after setting the speed
      checkLimits();
    }
  
    @Override
    public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.addDoubleProperty("Position", this::getElevatorPosition, null);
      builder.addDoubleProperty("kElevatorMaxSpeed", () -> kElevatorMaxSpeed, (x) -> {
        kElevatorMaxSpeed = x; updateCfg();
      });
      builder.addDoubleProperty("kElevatorMaxAccel", () -> kElevatorMaxAccel, (x) -> {
      kElevatorMaxAccel = x; updateCfg();
    });
    builder.addDoubleProperty("Tolerance", () -> kTolerance, (x) -> {
      kTolerance = x;
    });
    builder.addDoubleProperty("PickupLevel", ()-> kPickupLevel, (x)-> {
      kPickupLevel = x;
    });
    builder.addDoubleProperty("Level1Trough", () -> kLevel1Trough, (x) -> {
      kLevel1Trough = x;
    });
    builder.addDoubleProperty("Level2", () -> kLevel2, (x) -> {
      kLevel2 = x;
    });
    builder.addDoubleProperty("Level3", () -> kLevel3, (x) -> {
      kLevel3 = x;
    });
    builder.addDoubleProperty("Level4", () -> kLevel4, (x) -> {
      kLevel4 = x;
    });
    builder.addBooleanProperty("AtPickup", this::isNearPickupLevel, null);
    builder.addBooleanProperty("AtLevel1", this::isNearLevel1, null);
    builder.addBooleanProperty("AtLevel2", this::isNearLevel2, null);
    builder.addBooleanProperty("AtLevel3", this::isNearLevel3, null);
    builder.addBooleanProperty("AtLevel4", this::isNearLevel4, null);
    builder.addDoubleProperty("elevator kP", () -> kP, (x) -> {
      kP = x; updateCfg();
    });
    builder.addDoubleProperty("elevator kI", () -> kI, (x) -> {
      kI = x; updateCfg();
    });
    builder.addDoubleProperty("elevator kD", () -> kD, (x) -> {
      kD = x; updateCfg();
    });
  }

  public void stop() {
    setPosition(getElevatorPosition());
  }
}
