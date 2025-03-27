/*
 * TODO Elevator bringup tasks
 * 
 * Confirm positive left speed sends elevator up
 * Confirm negative right speed sends elevator up
 * Confirm encoder increases as elevator goes up
 * Find upper and lower position limits
 * Tune PID constants
 * Tune profile velocity and accel values
 * Find maximum up speed with no cable slack (fully loaded)
 */

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  public SparkMax elevatorMotor;
  public ProfiledPIDController PID;
  public DigitalInput elevatorHighLimitSwitch;
  public DigitalInput elevatorLowLimitSwitch;
  public RelativeEncoder encoder;

  public double kLimitLow = ElevatorConstants.kLimitLow;
  public double kLimitHigh = ElevatorConstants.kLimitHigh;
  public double kSoftLimitLow = ElevatorConstants.kSoftLimitLow;
  public double kSoftLimitHigh = ElevatorConstants.kSoftLimitHigh;
  public double kLoadPosition = ElevatorConstants.kLoadPosition;
  public double kPickupLevel = ElevatorConstants.kPickupLevel;
  public double kLevel2 = ElevatorConstants.kLevel2;
  public double kLevel3 = ElevatorConstants.kLevel3;
  public double kLevel4 = ElevatorConstants.kLevel4;
    public double kAlgaeLevel = ElevatorConstants.kAlgaeLevel;
    
      public Elevator(int MotorID, int highSwitchId, int lowSwitchId) {
        elevatorMotor = new SparkMax(MotorID, MotorType.kBrushless);
        elevatorMotor.configure(new SparkMaxConfig().inverted(true)
                                                    .idleMode(IdleMode.kBrake),
                                ResetMode.kResetSafeParameters,
                                PersistMode.kNoPersistParameters
                               );
        elevatorHighLimitSwitch = new DigitalInput(highSwitchId);
        elevatorLowLimitSwitch = new DigitalInput(lowSwitchId);
        encoder = elevatorMotor.getEncoder();
        encoder.setPosition(kLimitLow);
  
        PID = new ProfiledPIDController(ElevatorConstants.kP, 
                                        ElevatorConstants.kI, 
                                        ElevatorConstants.kD,
                                        new Constraints(ElevatorConstants.kElevatorMaxSpeed,
                                                        ElevatorConstants.kElevatorMaxAccel));
        PID.setTolerance(ElevatorConstants.kTolerance);
        PID.reset(getElevatorPosition());
  
        stop();
      }
    
      public double getElevatorSpeed() {
        return encoder.getVelocity();
      }
    
      public double getElevatorPosition() {
        // returns relative encoder position from one of the motors in full rotations
        return encoder.getPosition();
      }
    
      public void changeSetPoint(double delta){
        double p = getElevatorPosition();
        setPosition(p + delta);
      }
  
      public Command testMoveCommand(double speed){
        return new FunctionalCommand(()->{elevatorMotor.set(speed);
                                          System.out.println("Speed: " + speed);},
                                     ()->{},
                                     (x)->{elevatorMotor.set(0.0);
                                           System.out.println("stop");},
                                     ()->false,
                                     this 
                                     );
      }
  
  
      public void resetElevatorPositionEncoder(double position) {
        encoder.setPosition(position);
        PID.reset(position);
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
      }
      
      public void setPosition(double setpoint) {
        setpoint = MathUtil.clamp(setpoint, kSoftLimitLow, kSoftLimitHigh);
        PID.setGoal(setpoint);
      }
    
      public boolean isNear(double position) {
        return MathUtil.isNear(position, getElevatorPosition(), PID.getPositionTolerance());
      }
    
      public Command setPostionCommand(double setpoint){
        return new FunctionalCommand(()->setPosition(setpoint),
                                   ()->{},
                                   (x)->{},
                                   ()->isNear(setpoint),
                                   this
                                  );
      }
  
    public Command loadPosCommand() { return setPostionCommand(kLoadPosition); }
    public Command level2Command() { return setPostionCommand(kLevel2); }
    public Command level3Command() { return setPostionCommand(kLevel3); }
    public Command level4Command() { return setPostionCommand(kLevel4); }
  
    public boolean atSetpoint(){
      return PID.atGoal();
    }
    
    
      @Override
      public void periodic() {
        if (!DriverStation.isTest()){// todo I bet we need some feedforward here
        double speed = PID.calculate(encoder.getPosition());
        elevatorMotor.set(speed);
        // important to check the limits after setting the speed
        checkLimits();}
      }
    
      @Override
      public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Position", this::getElevatorPosition, this::resetElevatorPositionEncoder);
        builder.addDoubleProperty("Tolerance", () -> PID.getPositionTolerance(),
                                                   (x) -> PID.setTolerance(x));
      builder.addDoubleProperty("PickupLevel", ()-> kPickupLevel, (x)-> {
        kPickupLevel = x;
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
      builder.addBooleanProperty("AtPickup", ()->isNear(kLoadPosition), null);
      builder.addBooleanProperty("AtLevel2", ()->isNear(kLevel2), null );
      builder.addBooleanProperty("AtLevel3", ()->isNear(kLevel3), null );
      builder.addBooleanProperty("AtLevel4", ()->isNear(kLevel4), null );
      builder.addDoubleProperty("motor output", elevatorMotor::getAppliedOutput, null);
    }
  
    public void stop() {
      setPosition(getElevatorPosition());
    }
  
  public Command algaeLevelCommand() {
      // TODO Auto-generated method stub
      return setPostionCommand(kAlgaeLevel);
}
}
