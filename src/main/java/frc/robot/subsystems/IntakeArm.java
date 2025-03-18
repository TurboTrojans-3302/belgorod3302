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

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;

public class IntakeArm extends SubsystemBase {

  private SparkMax m_armLeftSparkMax;
  private SparkMax m_armRightSparkMax;
  private SparkMaxSim m_armSparkMaxSim;
  private SparkAbsoluteEncoder m_ArmEncoderRight;
  private SparkAbsoluteEncoderSim m_ArmEncoderRightSim;
  private SparkAbsoluteEncoder m_ArmEncoderLeft;
  private SparkAbsoluteEncoderSim m_ArmEncoderLeftSim;
  private double m_armAngleOffset = IntakeConstants.armAngleOffset;
  public ProfiledPIDController m_PidControllerRight;
  public ProfiledPIDController m_PidControllerLeft;
  private ArmFeedforward m_Feedforward;
  private double kS = IntakeConstants.kS;
  private double kG = IntakeConstants.kG;
  private double kV = IntakeConstants.kV;
  private double kA = IntakeConstants.kA;
  private double kMaxArmAngle = IntakeConstants.MaxArmAngle;
  private double kMinArmAngle = IntakeConstants.MinArmAngle;
  private double kFloorPosition = IntakeConstants.kFloorPosition;
  private double kElevatorPosition = IntakeConstants.kElevatorPosition;
  private double kTroughPosition = IntakeConstants.kTroughPosition;
  private static final double kGearRatio = 100.0;
  private static final double kVelocityConversionFactor = (360.0 / 60.0) / kGearRatio; // converts RPM to deg/sec
  private static final double kPositionConversionFactor = 360.0 / kGearRatio; // converts Revolutions to degrees

  private static final SparkMaxConfig leftSparkConfig = new SparkMaxConfig();
  private static final SparkMaxConfig rightSparkConfig = new SparkMaxConfig();
  static {
    leftSparkConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50);
    leftSparkConfig.encoder
          .positionConversionFactor(kPositionConversionFactor) 
          .velocityConversionFactor(kVelocityConversionFactor);

    rightSparkConfig
          .inverted(true)
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50)
          .follow(Constants.CanIds.intakeArmLeftMotorID, true);
    rightSparkConfig.encoder
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
  private double ff = 0;

  private SingleJointedArmSim m_sim;

  /** Creates a new IntakeArm. */
  public IntakeArm() {

    m_armLeftSparkMax = new SparkMax(Constants.CanIds.intakeArmLeftMotorID, MotorType.kBrushed);
    m_armLeftSparkMax.configure(leftSparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    m_armRightSparkMax = new SparkMax(Constants.CanIds.intakeArmRightMotorID, MotorType.kBrushed);
    m_armRightSparkMax.configure(rightSparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    
    m_ArmEncoderRight = m_armRightSparkMax.getAbsoluteEncoder();
    m_ArmEncoderLeft  = m_armLeftSparkMax.getAbsoluteEncoder();
    m_PidControllerRight = new ProfiledPIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, 
                                                new Constraints(IntakeConstants.kMaxVelocity, IntakeConstants.kMaxAcceleration));
    m_PidControllerLeft = new ProfiledPIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, 
                                                new Constraints(IntakeConstants.kMaxVelocity, IntakeConstants.kMaxAcceleration));
    resetFeedForward();
    m_velocityFilter = LinearFilter.singlePoleIIR(0.1, Robot.kDefaultPeriod);
    m_lastArmAngle = getArmAngleRightDegrees();

    m_ArmEncoderRightSim = new SparkAbsoluteEncoderSim(m_armRightSparkMax);
    m_ArmEncoderLeftSim  = new SparkAbsoluteEncoderSim(m_armLeftSparkMax);
    DCMotor plant = DCMotor.getAndymark9015(2);
    m_armSparkMaxSim = new SparkMaxSim(m_armLeftSparkMax, plant);

    m_sim = new SingleJointedArmSim(plant,
                                    kGearRatio,
                                    kMoment,
                                    kArmLength,
                                    Math.toRadians(kMinArmAngle),
                                    Math.toRadians(kMaxArmAngle),
                                    true,
                                    0
                                    );
    m_sim.setState( 0, 0);


  }

  private void resetFeedForward() {
    m_Feedforward = new ArmFeedforward(kS, kG, kV, kA);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double newAngle = getArmAngleRightDegrees();
    double vel = (newAngle - m_lastArmAngle) / Robot.kDefaultPeriod;
    m_armVelocity = m_velocityFilter.calculate(vel);
    m_lastArmAngle = newAngle;

    pidLeft = m_PidControllerRight.calculate(newAngle);
    State intermediateLeft = m_PidControllerRight.getSetpoint();
    ff = m_Feedforward.calculate(Math.toRadians(intermediateLeft.position),
                                        Math.toRadians(intermediateLeft.velocity));

    pidRight = m_PidControllerRight.calculate(newAngle);
    State intermediateRight = m_PidControllerRight.getSetpoint();
    ff = m_Feedforward.calculate(Math.toRadians(intermediateRight.position),
                                        Math.toRadians(intermediateRight.velocity));

    if(!DriverStation.isTest()){
      m_armLeftSparkMax.set( (pidLeft + ff));
      m_armRightSparkMax.set( (pidRight + ff));
    }
  }

  public void setPositionAngleSetpoint(double angle) {
    double setpoint = MathUtil.clamp(angle, kMinArmAngle, kMaxArmAngle);
    m_PidControllerRight.setGoal(setpoint);
  }

  public boolean atSetpoint(){
    return m_PidControllerRight.atSetpoint();
  }

  public void changeSetPoint(double delta){
    double p = getPositionAngleSetpoint();
    setPositionAngleSetpoint(p + delta);
  }

  public double getPositionAngleSetpoint() {
    return m_PidControllerRight.getGoal().position;
  }

  public double getArmAngleRightDegrees() {
    return (m_ArmEncoderRight.getPosition() * 360.0) + m_armAngleOffset;
  }

  public double getArmAngleLeftDegrees() {
    return (m_ArmEncoderLeft.getPosition() * 360.0) + m_armAngleOffset;
  }

  public double getArmAngleVelocity() {
    return m_armVelocity;
  }

  public void floorPosition(){ setPositionAngleSetpoint(kFloorPosition); }
  public void troughPosition(){ setPositionAngleSetpoint(kTroughPosition); }
  public void elevatorPosition(){ setPositionAngleSetpoint(kElevatorPosition); }

  public Command testCommand(double speed){
    return new FunctionalCommand(
                          ()->{m_armLeftSparkMax.set(speed);
                               m_armRightSparkMax.set(speed);},
                          ()->{},
                          (x)->{m_armLeftSparkMax.set(0.0);
                                m_armRightSparkMax.set(0.0);},
                          ()->false,
                          this
                          );
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("ArmAngleR", this::getArmAngleRightDegrees, null);
    builder.addDoubleProperty("ArmAngleL", this::getArmAngleLeftDegrees, null);
    builder.addDoubleProperty("ArmAngleOffset", () -> m_armAngleOffset, (x) -> {
      m_armAngleOffset = x;
    });
    builder.addDoubleProperty("kS", () -> kS, (x) -> {
      kS = x;
      resetFeedForward();
    });
    builder.addDoubleProperty("kG", () -> kG, (x) -> {
      kG = x;
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
    builder.addStringProperty("ff", ()->String.format("%.2f", ff), null);
    builder.addDoubleProperty("motorVoltage", ()->m_armLeftSparkMax.getAppliedOutput()*12, null);
  }

  @Override
  public void simulationPeriodic(){
    m_sim.setInputVoltage(m_armSparkMaxSim.getAppliedOutput() * 12.0);
    m_sim.update(Robot.kDefaultPeriod);
    m_armSparkMaxSim.iterate(Math.toDegrees(m_sim.getVelocityRadPerSec()), 12.0, Robot.kDefaultPeriod);
    m_ArmEncoderRightSim.setPosition(m_sim.getAngleRads()/6.28318);
    m_ArmEncoderLeftSim.setPosition(m_sim.getAngleRads()/6.28318);
  }
}
