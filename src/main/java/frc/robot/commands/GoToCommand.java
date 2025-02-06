// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.SwerveUtils;

public class GoToCommand extends Command {

  private final double DISTANCE_TOLERANCE = 0.050;
  private final double HEADING_TOLERANCE = 2.0;
  private final double dT = Robot.kDefaultPeriod;

  private Pose2d m_dest;
  private Transform2d m_delta;
  private DriveSubsystem m_drive;
  private TrapezoidProfile m_trapezoid;
  private boolean m_relativeFlag;

  private GoToCommand(DriveSubsystem drive){
    m_drive = drive;
    addRequirements(m_drive);
    m_trapezoid = new TrapezoidProfile(new Constraints(m_drive.getMaxSpeedLimit() * 0.5,
                                                       m_drive.getMaxSpeedLimit() * 1.0 )); //todo use full speed;
  }

  public GoToCommand(DriveSubsystem drive, Pose2d dest){
    this(drive);
    m_dest = dest;
    m_relativeFlag = false;
  }

  public static GoToCommand absolute(DriveSubsystem drive, double x, double y, double heading){
    Pose2d dest = new Pose2d(x, y, Rotation2d.fromDegrees(heading));
    return new GoToCommand(drive, dest);
  }

  public static GoToCommand relative(DriveSubsystem drive, double x, double y, double theta){
    Transform2d delta = new Transform2d(x, y, Rotation2d.fromDegrees(theta));
    return new GoToCommand(drive, delta);
  }

  public GoToCommand(DriveSubsystem drive, Transform2d delta){
    this(drive);
    m_delta = delta;
    m_relativeFlag = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if(m_relativeFlag){
      Translation2d dest_translation = m_delta.getTranslation();
      Rotation2d dest_rotation = m_drive.getPose().getRotation().plus(m_delta.getRotation());
      m_dest = new Pose2d(dest_translation, dest_rotation);
    }
    System.out.println("Starting go to: " + m_dest);
    RobotContainer.getInstance().m_field.getObject("dest").setPose(m_dest);
  }

  private Translation2d translation2dest(){
    return m_dest.minus(m_drive.getPose()).getTranslation();
  }

  private double distance(){
    return translation2dest().getNorm();
  }

  private double deltaHeading(){
    return SwerveUtils.angleDeltaDeg(m_drive.getHeading(), m_dest.getRotation().getDegrees());
  }

  private double speedTowardTarget(){
    Translation2d botDirection = m_drive.getVelocityVector();
    Translation2d targetDirection = translation2dest();

    if(botDirection.getNorm() <= 1e-6) {
      return 0.0;
    } else if(targetDirection.getNorm() <= 1e-6){
      return -m_drive.getSpeed();
    }

    Double difference = targetDirection.getAngle().getRadians() - botDirection.getAngle().getRadians();
    return m_drive.getSpeed() * Math.cos(difference);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("Speed: " + m_drive.getSpeed() + " translation2dest(): " + translation2dest());

    State currentState = new State(0.0, speedTowardTarget());
    State goalState = new State(distance(), 0.0);
    
    double speed = m_trapezoid.calculate(dT, currentState, goalState).velocity;

    Translation2d unitTranslation = translation2dest().div(translation2dest().getNorm());
    double turn = m_drive.turnToHeading(m_dest.getRotation().getDegrees());


    m_drive.driveFieldOriented(unitTranslation.times(speed), turn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setAll(0.0, 0.0);
    System.out.println("End go to: " + m_drive.getPose());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distance() < DISTANCE_TOLERANCE && 
           Math.abs(deltaHeading()) < HEADING_TOLERANCE;
  }

}
