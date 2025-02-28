// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;
import java.util.function.BooleanSupplier;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.CanIds;
import frc.robot.Constants.DigitalIO;
import frc.robot.commands.DriveToAprilTag;
import frc.robot.commands.GoToCommand;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.NavigateToTag;
import frc.robot.commands.OrbitAroundReef;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TestDrive;
import frc.robot.commands.TurnToAprilTag;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArm;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static boolean ELEVATOR_ENABLE = false;
  private static boolean INTAKE_ENABLE = false;
  private static boolean INTAKE_ARM_ENABLE = false;
  private static boolean GRIPPER_ENABLE = false;
  private static boolean CLIMBERS_ENABLE = false;

  private static RobotContainer instance;

  // The robot's subsystems
  public DriveSubsystem m_robotDrive;
  public Navigation m_nav;
  public Elevator m_elevator;
  public Intake m_intake;
  public IntakeArm m_intakeArm;
  public Gripper m_gripper;
  public Climbers m_climbers;

  private SendableChooser<Command> m_autonomousChooser;
  private SendableChooser<Pose2d> m_startPosChooser;

  private final REVBlinkinLED m_BlinkinLED;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_copilotController = new XboxController(OIConstants.kCopilotControllerPort);

  private int targetTagId = 0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    instance = this;

    // The robot's subsystems
    m_robotDrive = new DriveSubsystem();
    SmartDashboard.putData("DriveSubsystem", m_robotDrive);
    m_nav = new Navigation(m_robotDrive);
    SmartDashboard.putData("Navigation", m_nav);
    if (ELEVATOR_ENABLE) {
      m_elevator = new Elevator(CanIds.kLeftMotorElevatorCanId,
          CanIds.kRightMotorElevatorCanId,
          DigitalIO.kElevatorHighLimitSwitchId,
          DigitalIO.kElevatorLowLimitSwitchId);
      SmartDashboard.putData("Elevator", m_elevator);
    }
    if (INTAKE_ENABLE) {
      m_intake = new Intake(CanIds.kIntakeMotorCanId, DigitalIO.kIntakeLimitSwitchId);
      SmartDashboard.putData("Intake", m_intake);
    }
    if (INTAKE_ARM_ENABLE) {
      m_intakeArm = new IntakeArm();
      SmartDashboard.putData("IntakeArm", m_intakeArm);
    }
    if (GRIPPER_ENABLE) {
      m_gripper = new Gripper(CanIds.kGripperMotorCanId,
          CanIds.kGripperExtensionMotorCanId,
          DigitalIO.kGripperClosedSwitchId,
          DigitalIO.kGripperFullyRetractedSwitchId,
          DigitalIO.kGripperObjectDetectedSwitchId);
      SmartDashboard.putData("Gripper", m_gripper);
    }
    if (CLIMBERS_ENABLE) {
      m_climbers = new Climbers(CanIds.kClimberLeftMotorCanId,
          CanIds.kClimberRightMotorCanId,
          DigitalIO.kClimberLimitSwitchId);
      SmartDashboard.putData("Climbers", m_climbers);
    }

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    Command teleopCommand = new TeleopDrive(m_robotDrive, m_driverController);
    m_robotDrive.setDefaultCommand(teleopCommand);
    SmartDashboard.putData("TeleopCommand", teleopCommand);
   // m_robotDrive.setDefaultCommand(new TestDrive(m_robotDrive, m_driverController));

    m_BlinkinLED = new REVBlinkinLED(Constants.BLINKIN_LED_PWM_CHANNEL);
  }

  public static RobotContainer getInstance() {
    return instance;
  }

  private void configureButtonBindings() {

    /**
     * Driver's Controller
     */
    new Trigger(()->{ return m_driverController.getPOV() == 0; })
      .onTrue(new RunCommand(()->{ targetTagId = (int) LimelightHelpers.getFiducialID("limelight"); }));
    new Trigger(()->{ return m_driverController.getPOV() == 180; })
      .whileTrue(Commands.defer(()->new NavigateToTag(m_robotDrive, m_nav, ()->targetTagId ),
                                Set.of(m_robotDrive, m_nav)));

    new JoystickButton(m_driverController, XboxController.Button.kB.value)
      .whileTrue(new OrbitAroundReef(m_robotDrive, m_nav, 1.0));
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
      .whileTrue(new OrbitAroundReef(m_robotDrive, m_nav, -1.0));

    if(INTAKE_ENABLE){
        new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
            .whileTrue(new RunCommand( () -> m_intake.in(), m_intake));
        new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
            .whileTrue(new RunCommand( () -> m_intake.out(), m_intake));
    }

    /**
     * Copilot's Controller
     *
     */        
    if (ELEVATOR_ENABLE) {
      new JoystickButton(m_copilotController, XboxController.Button.kA.value)
          .onTrue(new MoveElevator(m_elevator,
              Constants.ElevatorConstants.kLevel1Trough,
              Constants.ElevatorConstants.kElevatorAutoSpeedToLevel));

      new JoystickButton(m_copilotController, XboxController.Button.kB.value)
          .onTrue(new MoveElevator(m_elevator, Constants.ElevatorConstants.kLevel2,
              Constants.ElevatorConstants.kElevatorAutoSpeedToLevel));

      new JoystickButton(m_copilotController, XboxController.Button.kX.value)
          .onTrue(new MoveElevator(m_elevator, Constants.ElevatorConstants.kLevel3,
              Constants.ElevatorConstants.kElevatorAutoSpeedToLevel));

      new JoystickButton(m_copilotController, XboxController.Button.kY.value)
          .onTrue(new MoveElevator(m_elevator, Constants.ElevatorConstants.kLevel4,
              Constants.ElevatorConstants.kElevatorAutoSpeedToLevel));

      // get dpad position as a boolean (they are automatically returned by getPOV()
      // as an exact value)
      BooleanSupplier dpadUp = () -> m_copilotController.getPOV() == 0;
      BooleanSupplier dpadDown = () -> m_copilotController.getPOV() == 180;

      // convert booleansupplier into triggers so the whileTrue() method can be called
      // upon them
      Trigger elevatorUp = new Trigger(dpadUp);
      Trigger elevatorDown = new Trigger(dpadDown);

      // dpad causes the elevator to go up/down slowly during teleop
      elevatorUp.whileTrue(new MoveElevator(m_elevator,
          Constants.ElevatorConstants.kLevel4,
          Constants.ElevatorConstants.kElevatorPrecisionControlSpeed));
      elevatorDown.whileTrue(new MoveElevator(m_elevator, 0,
          Constants.ElevatorConstants.kElevatorPrecisionControlSpeed));


    }

    if (GRIPPER_ENABLE){
      new JoystickButton(m_copilotController, XboxController.Button.kRightBumper.value)
      .onTrue(new InstantCommand(() ->  m_gripper.toggleGripper()));
      Trigger extensionOut = new Trigger( () -> (m_copilotController.getLeftTriggerAxis() > 0.8));
      Trigger extensionIn = new Trigger(() -> (m_copilotController.getRightTriggerAxis() > 0.8));

      extensionOut.onTrue(new InstantCommand(() -> m_gripper.extendGripper()));
      extensionIn.onTrue(new InstantCommand(() -> m_gripper.retractGripper()));
  
    }
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

  public void setLED(double value) {
    m_BlinkinLED.set(value);
  }

  /*
   * called once when is set to Red by the DriverStation
   */
  public void initRed() {
    m_autonomousChooser = AutonMenus.getRed();
    SmartDashboard.putData("Auton Command", m_autonomousChooser);

    m_startPosChooser = StartPositions.getRed();
    SmartDashboard.putData("Start Position", m_startPosChooser);
    m_startPosChooser.onChange(this::setStartPosition);
  }

  /*
   * called once when is set to Blue by the DriverStation
   */
  public void initBlue() {
    m_autonomousChooser = AutonMenus.getBlue();
    SmartDashboard.putData("Auton Command", m_autonomousChooser);

    m_startPosChooser = StartPositions.getBlue();
    SmartDashboard.putData("Start Position", m_startPosChooser);
    m_startPosChooser.onChange(this::setStartPosition);
  }

  private void setStartPosition(Pose2d pose) {
    if (DriverStation.isDisabled()) {
      m_nav.resetOdometry(pose);
    }
  }

}
