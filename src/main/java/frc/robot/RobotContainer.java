// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveToAprilTag;
import frc.robot.commands.GoToCommand;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TurnToAprilTag;
import frc.robot.subsystems.DriveSubsystem;

 

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  
  //private final ShuffleboardTab m_shuffleboardTab;
  private final SendableChooser<Command> m_autonomousChooser;
  private final SendableChooser<Pose2d> m_startPosChooser;

  private final REVBlinkinLED m_BlinkinLED;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_copilotController = new XboxController(OIConstants.kCopilotControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(new TeleopDrive(m_robotDrive, m_driverController));


    //m_shuffleboardTab = Shuffleboard.getTab("Game");
    
    m_autonomousChooser = new SendableChooser<Command>();
    m_autonomousChooser.setDefaultOption("turn to april tag B 10", new TurnToAprilTag(m_robotDrive, 10));
    m_autonomousChooser.addOption("turn to april tag 1", new TurnToAprilTag(m_robotDrive, 1));
    m_autonomousChooser.addOption("turn to april tag 11", new TurnToAprilTag(m_robotDrive, 11));
    m_autonomousChooser.addOption("Drive to april tag 1", new DriveToAprilTag(m_robotDrive, 1));
    m_autonomousChooser.addOption("GoTo 1, 0, 0", GoToCommand.absolute(m_robotDrive, 1.0, 0, 0));
    m_autonomousChooser.addOption("GoTo 1, 1, 0", GoToCommand.absolute(m_robotDrive, 1.0, 1.0, 0));
    SmartDashboard.putData("Auton Command", m_autonomousChooser);

    m_startPosChooser = new SendableChooser<Pose2d>();
    m_startPosChooser.setDefaultOption("ZeroZero", Constants.FieldConstants.ZeroZero);
    m_startPosChooser.addOption("Left +30", new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(30.0)));
    m_startPosChooser.addOption("Right -30", new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-30.0)));
   // m_shuffleboardTab.add("Start Position", m_startPosChooser);

    m_BlinkinLED = new REVBlinkinLED(Constants.BLINKIN_LED_PWM_CHANNEL);


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    new JoystickButton(m_driverController, XboxController.Button.kRightStick.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    
    
    // new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
    //     .whileTrue(new RunCommand(() -> {
    //                                       if(m_harvester.getArmAngle() > 90){
    //                                           m_harvester.setIntakeSpeed(Constants.harvesterConstants.outSpeed);
    //                                       }else{
    //                                           m_harvester.setIntakeSpeed(Constants.harvesterConstants.outSpeedSlow);
    //                                       }
    //                                     }, m_harvester));                             
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonomousChooser.getSelected();
  }

  public Pose2d getStartPosition() {
    return m_startPosChooser.getSelected();
  }

  public void setStartPosition() {
    m_robotDrive.resetOdometry(getStartPosition());
  }

  public void setLED(double value) {
    m_BlinkinLED.set(value);
  }
}
