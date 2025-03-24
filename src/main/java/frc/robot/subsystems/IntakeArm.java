/*
 * TODO intakeArm bringup steps
 * confirm positive speed == arm go up
 * confirm L and R motors are going the same way
 * confirm arm-go-up means and increase in encoder value
 * confirm encoder value is degrees, or recalculate kPositionConversionFactor
 * adjust m_armAngleOffset so that 0deg is maximum torque position 
 * Calibrate kMaxArmAngle and kMinArmAngle
 * Calibrate kFloorPosition, kElevatorPosition and kTroughPosition?
 * Set kG
 * Tune PID (keep profile constraints very high)
 * Tune profile constraints
 * 
 */

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;

public class IntakeArm extends SubsystemBase {

  private SparkMax m_armLeftSparkMax;
  private RelativeEncoder m_ArmEncoderLeft;
  private double m_armAngleOffsetLeft = IntakeConstants.armAngleOffsetLeft;
  public PIDController m_PidControllerLeft;
  private ArmFeedforward m_FeedforwardLeft;
  private double kS = IntakeConstants.kS;
  private double kGLeft = IntakeConstants.kGLeft;
  private double kV = IntakeConstants.kV;
  private double kA = IntakeConstants.kA;
  private double kMaxArmAngle = IntakeConstants.MaxArmAngle;
  private double kMinArmAngle = IntakeConstants.MinArmAngle;
  private double kFloorPosition = IntakeConstants.kFloorPosition;
  private double kElevatorPosition = IntakeConstants.kElevatorPosition;
  private double kTroughPosition = IntakeConstants.kTroughPosition;
  private static final double kGearRatio = 100.0;
  private static final double kPositionConversionFactor = 180.0; // converts to degrees
  private static final double kVelocityConversionFactor = kPositionConversionFactor/ 60.0; // converts RPM to deg/sec

  private static final SparkMaxConfig leftSparkConfig = new SparkMaxConfig();
  static {
    leftSparkConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50);
    leftSparkConfig.encoder
          .positionConversionFactor(kPositionConversionFactor) 
          .velocityConversionFactor(kVelocityConversionFactor);
}

  // simulation constants
  private final double kMoment = SingleJointedArmSim.estimateMOI(0.355, 9.1);
  private final double kArmLength = .355;

  private LinearFilter m_velocityFilter;
  private double m_lastArmAngle;
  private double m_armVelocity = 0.0;
  private double pidLeft = 0;
  private double pidRight = 0;
  private double ffLeft = 0;
  private double ffRight = 0;

  private SingleJointedArmSim m_sim;
  private double kPositionTolerance = IntakeConstants.kPositionTolerance;
  
    /** Creates a new IntakeArm. */
    public IntakeArm() {
  
      m_armLeftSparkMax = new SparkMax(Constants.CanIds.intakeArmLeftMotorID, MotorType.kBrushed);
      m_armLeftSparkMax.configure(leftSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  
      m_ArmEncoderLeft  = m_armLeftSparkMax.getEncoder();
      m_ArmEncoderLeft.setPosition(kMaxArmAngle);

      m_PidControllerLeft = new PIDController(IntakeConstants.kPleft, IntakeConstants.kI, IntakeConstants.kD);
      m_PidControllerLeft.reset();
      resetFeedForward();
      m_velocityFilter = LinearFilter.singlePoleIIR(0.1, Robot.kDefaultPeriod);
      m_lastArmAngle = getArmAngleLeftDegrees();
      
      stop();
    }
  
    private void resetFeedForward() {
      m_FeedforwardLeft = new ArmFeedforward(kS, kGLeft, kV, kA);
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      double newAngle = getArmAngleLeftDegrees();
      double vel = (newAngle - m_lastArmAngle) / Robot.kDefaultPeriod;
      m_armVelocity = m_velocityFilter.calculate(vel);
      m_lastArmAngle = newAngle;
  
      pidLeft = m_PidControllerLeft.calculate(newAngle);
      ffLeft = m_FeedforwardLeft.calculate(Math.toRadians(getArmAngleLeftDegrees()), 0.0);

      if(!DriverStation.isTest() && DriverStation.isEnabled()){
        m_armLeftSparkMax.set( (pidLeft + ffLeft));
      }
    }
  
    public void setPositionAngleSetpoint(double angle) {
      double setpoint = MathUtil.clamp(angle, kMinArmAngle, kMaxArmAngle);
      m_PidControllerLeft.setSetpoint(setpoint);
    }
  
    public boolean atSetpoint(){
      return m_PidControllerLeft.atSetpoint();
    }
  
    public void changeSetPoint(double delta){
      double p = getPositionAngleSetpoint();
      setPositionAngleSetpoint(p + delta);
    }
  
    public double getPositionAngleSetpoint() {
      return m_PidControllerLeft.getSetpoint();
    }
  
    public double getArmAngleLeftDegrees() {
      return m_ArmEncoderLeft.getPosition() + m_armAngleOffsetLeft;
    }
  
    public double getArmAngleVelocity() {
      return m_armVelocity;
    }
  
    public void floorPosition(){ setPositionAngleSetpoint(kFloorPosition); }
    public void troughPosition(){ setPositionAngleSetpoint(kTroughPosition); }
    public void elevatorPosition(){ setPositionAngleSetpoint(kElevatorPosition); }

    public Command setPositionCommand(double position){
      return new FunctionalCommand(
                            ()->setPositionAngleSetpoint(position),
                            ()->{},
                            (x)->{},
                            ()->atSetpoint(),
                            this
                            );
    }

    public Command changePositionCommand(double delta){
      return new InstantCommand(()->changeSetPoint(delta));
    }
  
    public Command testCommand(double speed){
      return new FunctionalCommand(
                            ()->{m_armLeftSparkMax.set(speed);},
                            ()->{},
                            (x)->{m_armLeftSparkMax.set(0.0);},
                            ()->false,
                            this
                            );
    }

    public String getPositionLabel(){
      double pos = getArmAngleLeftDegrees();
      if(MathUtil.isNear(kFloorPosition, pos, kPositionTolerance)){
      return "Floor";
    }else if(MathUtil.isNear(kTroughPosition, pos, kPositionTolerance)){
      return "Trough";
    }else if(MathUtil.isNear(kElevatorPosition, pos, kPositionTolerance)){
      return "Elevator";
    }else{
      return String.format("%.1f", pos);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("ArmAngleL", this::getArmAngleLeftDegrees, null);
    builder.addDoubleProperty("ArmAnglOffstL", () -> m_armAngleOffsetLeft, (x) -> {
      m_armAngleOffsetLeft = x;
    });
    builder.addDoubleProperty("kS", () -> kS, (x) -> {
      kS = x;
      resetFeedForward();
    });
    builder.addDoubleProperty("kGLeft", () -> kGLeft, (x) -> {
      kGLeft = x;
      resetFeedForward();
    });
    builder.addDoubleProperty("kV", () -> kV, (x) -> {
      kV = x;
      resetFeedForward();
    });
    builder.addDoubleProperty("kA", () -> kA, (x) -> {
      kA = x;
      resetFeedForward();
    });
    builder.addDoubleProperty("kMinArmAngle", () -> kMinArmAngle, (x) -> {
      kMinArmAngle = x;
    });
    builder.addDoubleProperty("kMaxArmAngle", () -> kMaxArmAngle, (x) -> {
      kMaxArmAngle = x;
    });
    builder.addStringProperty("pid", ()->String.format("%.2f", pidLeft), null);
    builder.addStringProperty("ffLeft", ()->String.format("%.2f", ffLeft), null);
    builder.addDoubleProperty("LmotorOutput", ()->m_armLeftSparkMax.getAppliedOutput(), null);
    builder.addStringProperty("ArmAngleLabel", this::getPositionLabel, null);
  }


  public void stop() {
    m_PidControllerLeft.setSetpoint(getArmAngleLeftDegrees());
    m_armLeftSparkMax.set(0.0);  
  }
}
