// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.reflect.Method;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Eddie.DriveConstants;
import frc.robot.commands.OrbitAroundReef;

/*
 * Driver Xbox Controller
 * 
 * Left stick press: toggle slow drive, press (planned)
 * Right stick press: toggle field oriented, press (planned)
 * 
 * A button: 
 * B button: hold to orbit CCW around reef (planned)
 * X button: hold to orbit CW around reef (planned)
 * Y button: 
 * 
 * Up Arrow: Target April tag that is currently being looked at and sent to dashboard, press (planned)
 * Down Arrow: move to target apriltag, if not found send dashboard notification, hold (planned)
 * Right Arrow: orbit right, hold (CCW)
 * Left Arrow: orbit left, hold (CW)
 * 
 * Left Trigger - 
 * Right Trigger - 
 * 
 * Right Bumper - intake in (planned)
 * Left Bumper -  intake out (planned)
 * 
 */

/* 
 * Copilot Controller
 *
 * Left Stick 
 * Right Stick
 *
 * A Button - Trough preset position
 * B button
 * X button 
 * Y button - Level 4 elevator preset position
 *  
 *  Left Bumper - Load elevator routine (planned)
 *  Right Bumper 
 *
 *  Right Trigger - 
 *  Left Trigger
 *  
 *  hold DPad up - Elevator precision control up
 *  hold Dpad down - Elevator precision control down
 *
 *
 */

public class TeleopDrive extends Command {
  private DriveSubsystem m_robotDrive;
  private XboxController m_driverController;
  private boolean m_fieldOrientedEnable = true;
  private double orbitSpeed = DriveConstants.ORBIT_SPEED;
  DriveSubsystem drive;
  Navigation nav;

  /** Creates a new TeleopDrive. */
  public TeleopDrive(DriveSubsystem robotDrive, XboxController driverController) {
    m_driverController = driverController;
    m_robotDrive = robotDrive;
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_robotDrive.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedScale;
    
    if (m_driverController.getLeftBumperButton()) {
      speedScale = 0.5;
    } else {
      speedScale = 1.0;
    }

    double forward = stick2speed(speedScale * m_driverController.getLeftY());
    double leftward = stick2speed(speedScale * m_driverController.getLeftX());
    double rotate = stick2speed(speedScale * m_driverController.getRightX());

    if(forward == 0.0 && leftward == 0.0 && rotate == 0.0){

      if(m_driverController.getXButton()){
        new OrbitAroundReef(drive, nav, -0.5 * speedScale);
      }
      else if(m_driverController.getBButton()){
        new OrbitAroundReef(drive, nav, 0.5 * speedScale);
      }
      else{
        m_robotDrive.stop();
      }

      if(m_driverController.getPOV() == 90 && m_driverController.getXButton() == false && m_driverController.getBButton() == false){
        m_robotDrive.orbit(orbitSpeed * -speedScale);
      }
      else if(m_driverController.getPOV() == 270){
        m_robotDrive.orbit(orbitSpeed * speedScale);
      }else{
        m_robotDrive.stop();
      }

    } else {
      if(m_driverController.getRightBumperButton() || !m_fieldOrientedEnable) {
        m_robotDrive.driveRobotOriented(forward, leftward, rotate);
      } else {
        double reverse = (Robot.alliance == Alliance.Red) ? -1.0 : 1.0;
        m_robotDrive.driveFieldOriented(reverse * forward, reverse * leftward, rotate);
      }
    }
  }

  // applies deadband and scaling to raw stick value
  private double stick2speed(double stickValue) {
    return -Math.signum(stickValue) * Math.pow(MathUtil.applyDeadband(stickValue, OIConstants.kDriveDeadband), 2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty("FieldOrientedEnable", () -> m_fieldOrientedEnable, (x)->{m_fieldOrientedEnable = x;});
    builder.addDoubleProperty("Orbit Speed", ()-> orbitSpeed, (x)-> orbitSpeed = x);
  }
}
