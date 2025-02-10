// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveToAprilTag;
import frc.robot.commands.GoToCommand;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TurnToAprilTag;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static RobotContainer instance;

  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final Elevator m_elevator = new Elevator(Constants.ElevatorConstants.kLeftMotorElevatorCanId,
      Constants.ElevatorConstants.kRightMotorElevatorCanId,
      Constants.ElevatorConstants.kElevatorHighLimitSwitchId,
      Constants.ElevatorConstants.kElevatorLowLimitSwitchId);
  public Field2d m_field = new Field2d();

  // private final ShuffleboardTab m_shuffleboardTab;
  public final ShuffleboardTab m_elevatorTab = Shuffleboard.getTab("Elevator");
  public final ShuffleboardTab m_climbersTab = Shuffleboard.getTab("climbers and intake");
 

  GenericEntry level1Position;
  GenericEntry level2Position;
  GenericEntry level3Position;
  GenericEntry level4Position;
  GenericEntry processorPosition;
  GenericEntry elevatorMaxSpeed;
  GenericEntry elevatorPrecisionSpeed;
  GenericEntry elevatorAutoSpeed;
  
  GenericEntry climberMaxSpeed;
  GenericEntry climberAutoSpeed;

  GenericEntry intakeSpeedMax;

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
    instance = this;

    //elevator stuff
   /* DoubleSupplier elevatorAutoSpeed = () -> Constants.ElevatorConstants.kElevatorAutoSpeedToLevel;
    *DoubleSupplier elevatorPrecisionSpeed = () -> Constants.ElevatorConstants.kElevatorPrecisionControlSpeed;
    *DoubleSupplier elevatorMaxSpeed = () -> Constants.ElevatorConstants.kElevatorMaxSpeed;
    *DoubleSupplier level1Position = () -> Constants.ElevatorConstants.kLevel1Trough;
    *DoubleSupplier level2Position = () -> Constants.ElevatorConstants.kLevel2;
    *DoubleSupplier level3Position = () -> Constants.ElevatorConstants.kLevel3;
    *DoubleSupplier level4Position = () -> Constants.ElevatorConstants.kLevel4; 
   m_elevatorTab.addDouble("Elevator Auto Speed", elevatorAutoSpeed);
    m_elevatorTab.addDouble("Precision control speed", elevatorPrecisionSpeed);
    m_elevatorTab.addDouble("max speed", elevatorMaxSpeed);
    m_elevatorTab.addDouble("level 1 position", level1Position);
    m_elevatorTab.addDouble("level 2 position", level2Position);
    m_elevatorTab.addDouble("level 3 position", level3Position);
    m_elevatorTab.addDouble("level 4 position", level4Position); */

    
  
    level1Position = m_elevatorTab.add("level 1 position", Constants.ElevatorConstants.kLevel1Trough).getEntry();
    level2Position = m_elevatorTab.add("level 2 position", Constants.ElevatorConstants.kLevel2).getEntry();
    level3Position = m_elevatorTab.add("level 3 position", Constants.ElevatorConstants.kLevel3).getEntry();
    level4Position = m_elevatorTab.add("level 4 position", Constants.ElevatorConstants.kLevel4).getEntry();
    processorPosition = m_elevatorTab.add("processor position", Constants.ElevatorConstants.kProcessor).getEntry();
  
    elevatorMaxSpeed = m_elevatorTab.add("max speed", Constants.ElevatorConstants.kElevatorMaxSpeed).getEntry();
    elevatorPrecisionSpeed = m_elevatorTab.add("level 4 position", Constants.ElevatorConstants.kElevatorPrecisionControlSpeed).getEntry();
    elevatorAutoSpeed = m_elevatorTab.add("level 4 position", Constants.ElevatorConstants.kElevatorAutoSpeedToLevel).getEntry();

    climberAutoSpeed = m_climbersTab.add("Climber auto speed", Constants.ClimberConstants.climberAutoSpeed).getEntry();
    climberMaxSpeed = m_climbersTab.add("Climber auto speed", Constants.ClimberConstants.climberMaxSpeed).getEntry();
  
    intakeSpeedMax = m_climbersTab.add("intake speed max", Constants.IntakeConstants.intakeSpeedMax).getEntry();
  
  

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(new TeleopDrive(m_robotDrive, m_driverController));

    // m_shuffleboardTab = Shuffleboard.getTab("Game");

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

  public static RobotContainer getInstance() {
    return instance;
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

    new JoystickButton(m_copilotController, XboxController.Button.kA.value)
        .onTrue(new MoveElevator(m_elevator, Constants.ElevatorConstants.kLevel1Trough, Constants.ElevatorConstants.kElevatorAutoSpeedToLevel));

    new JoystickButton(m_copilotController, XboxController.Button.kB.value)
        .onTrue(new MoveElevator(m_elevator, Constants.ElevatorConstants.kLevel2, Constants.ElevatorConstants.kElevatorAutoSpeedToLevel));

    new JoystickButton(m_copilotController, XboxController.Button.kX.value)
        .onTrue(new MoveElevator(m_elevator, Constants.ElevatorConstants.kLevel3, Constants.ElevatorConstants.kElevatorAutoSpeedToLevel));

    new JoystickButton(m_copilotController, XboxController.Button.kY.value)
        .onTrue(new MoveElevator(m_elevator, Constants.ElevatorConstants.kLevel4, Constants.ElevatorConstants.kElevatorAutoSpeedToLevel));

        //get dpad position as a boolean (they are automatically returned by getPOV() as an exact value)
        BooleanSupplier dpadUp = () -> m_copilotController.getPOV() == 0;
        BooleanSupplier dpadDown = () -> m_copilotController.getPOV() == 180;

      //convert booleansupplier into triggers so the whileTrue() method can be called upon them
      Trigger elevatorUp = new Trigger(dpadUp);
      Trigger elevatorDown = new Trigger(dpadDown);

      //dpad causes the elevator to go up/down slowly during teleop
      elevatorUp.whileTrue(new MoveElevator(m_elevator, Constants.ElevatorConstants.kLevel4, Constants.ElevatorConstants.kElevatorPrecisionControlSpeed));
      elevatorDown.whileTrue(new MoveElevator(m_elevator, 0, Constants.ElevatorConstants.kElevatorPrecisionControlSpeed));
          
        };
        

        
        
  

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
