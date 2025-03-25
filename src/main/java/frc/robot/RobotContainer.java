// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CanIds;
import frc.robot.Constants.DigitalIO;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OIConstants.ButtonBox;
import frc.robot.commands.Coral;
import frc.robot.commands.ElevatorManualMove;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.NavigateToTag;
import frc.robot.commands.OrbitAroundReef;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TroughScore;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.Navigation;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static boolean ELEVATOR_ENABLE = true;
  private static boolean INTAKE_ENABLE = true;
  private static boolean INTAKE_ARM_ENABLE = true;
  private static boolean GRIPPER_ENABLE = true;
  private static boolean CLIMBERS_ENABLE = true;

  private static RobotContainer instance;

  // The robot's subsystems
  public DriveSubsystem m_robotDrive;
  public Navigation m_nav;
  public Elevator m_elevator;
  public Intake m_intake;
  public IntakeArm m_intakeArm;
  public Gripper m_gripper;
  public Climbers m_climbers;

  private SendableChooser<Command> m_autonomousChooser = new SendableChooser<Command>();
  private SendableChooser<Pose2d> m_startPosChooser = new SendableChooser<Pose2d>();

  private final REVBlinkinLED m_BlinkinLED;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_copilotController = new XboxController(OIConstants.kCopilotControllerPort);
  GenericHID m_buttonBoard = new GenericHID(OIConstants.kButtonBoardPort);
  //ReefController m_reefController = new ReefController(OIConstants.kReefControllerPort);

  public int targetTagId = 0;

  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    instance = this;
    
    CameraServer.startAutomaticCapture();
    
    // The robot's subsystems
    m_robotDrive = new DriveSubsystem();
    SmartDashboard.putData("DriveSubsystem", m_robotDrive);
    SmartDashboard.putData("Yaw PID", m_robotDrive.headingPidController);
    m_nav = new Navigation(m_robotDrive);
    SmartDashboard.putData("Navigation", m_nav);
    if (ELEVATOR_ENABLE) {
      m_elevator = new Elevator(CanIds.kElevatorCanId,
          DigitalIO.kElevatorHighLimitSwitchId,
          DigitalIO.kElevatorLowLimitSwitchId);
      SmartDashboard.putData("Elevator", m_elevator);
      SmartDashboard.putData("Elevator PID", m_elevator.PID);
    }
    if (INTAKE_ENABLE) {
      m_intake = new Intake(CanIds.kUpperIntakeMotorCanId, DigitalIO.kUpperIntakeLimitSwitchId);
      SmartDashboard.putData("Intake", m_intake);
    }
    if (INTAKE_ARM_ENABLE) {
      m_intakeArm = new IntakeArm();
      SmartDashboard.putData("IntakeArm", m_intakeArm);
      SmartDashboard.putData("ArmPIDleft", m_intakeArm.m_PidControllerLeft);
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
          CanIds.kClimberRightMotorCanId);
      SmartDashboard.putData("Climbers", m_climbers);
    }

    m_BlinkinLED = new REVBlinkinLED(Constants.BLINKIN_LED_PWM_CHANNEL);
  }

  public void setDefaultCommands(){
    // Configure default commands
    Command teleopCommand = new TeleopDrive(m_robotDrive, m_driverController);
    m_robotDrive.setDefaultCommand(teleopCommand);
    SmartDashboard.putData("TeleopCommand", teleopCommand);

  }

  public static RobotContainer getInstance() {
    return instance;
  }

  public void configureButtonBindings() {

    /**
     * Driver's Controller
     */
    new Trigger(() -> m_driverController.getPOV() == 0)
        .onTrue(new RunCommand(() -> {
          targetTagId = (int) LimelightHelpers.getFiducialID("limelight");
        }));
    new Trigger(() -> m_driverController.getPOV() == 180)
        .whileTrue(Commands.defer(() -> new NavigateToTag(m_robotDrive, m_nav, () -> targetTagId),
            Set.of(m_robotDrive, m_nav)));

    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .whileTrue(new OrbitAroundReef(m_robotDrive, m_nav, 1.0));
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(new OrbitAroundReef(m_robotDrive, m_nav, -1.0));
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(() -> {
          double stream = LimelightHelpers.getLimelightNTDouble("limelight", "stream");
          LimelightHelpers.setLimelightNTDouble("limelight", "stream",
              (stream == 0.0 ? 2.0 : 0.0));
        }));

    new JoystickButton(m_driverController, XboxController.Button.kA.value)
      .onTrue(new InstantCommand(()->{
        double newheading = m_robotDrive.getGyroAngleDegrees() + 180.0;
        if(newheading > 180.0){ newheading -= 180.0;}
        m_robotDrive.setGyroAngleDeg((newheading) );
      }));    

    if (INTAKE_ENABLE) {
      new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
          .onTrue(new RunCommand(() -> m_intake.in(), m_intake))
          .onFalse(new RunCommand(() -> m_intake.stop(), m_intake));
      new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
      .onTrue(new RunCommand(() -> m_intake.out(), m_intake))
      .onFalse(new RunCommand(() -> m_intake.stop(), m_intake));
}

    /**
     * Copilot's Controller
     *
     */
    if (ELEVATOR_ENABLE) {

      //TODO find elevator position for algae
      new JoystickButton(m_buttonBoard, ButtonBox.Left3)
          .onTrue(new MoveElevator(m_elevator, Constants.ElevatorConstants.kLoadPosition));
      

      new JoystickButton(m_buttonBoard, ButtonBox.Left2)
          .onTrue(new MoveElevator(m_elevator, Constants.ElevatorConstants.Algae));
      

          //just move elevator up to top position (for knocking off algae faster?)
      new JoystickButton(m_buttonBoard, ButtonBox.Left1)
          .onTrue(new MoveElevator(m_elevator, Constants.ElevatorConstants.kLevel4));
      

      // get dpad position as a boolean (they are automatically returned by getPOV()
      // as an exact value)
      BooleanSupplier dpadUp = () -> m_copilotController.getPOV() == 0;
      BooleanSupplier dpadDown = () -> m_copilotController.getPOV() == 180;

      // convert booleansupplier into triggers so the whileTrue() method can be called
      // upon them
      Trigger elevatorUp = new Trigger(dpadUp);
      Trigger elevatorDown = new Trigger(dpadDown);

      // dpad causes the elevator to go up/down slowly during teleop
      elevatorUp.whileTrue(new ElevatorManualMove(m_elevator, Constants.ElevatorConstants.kManualRate));
      elevatorDown.whileTrue(new ElevatorManualMove(m_elevator, -Constants.ElevatorConstants.kManualRate));

    }

    if (GRIPPER_ENABLE) {
      //TODO test extension controls
      new JoystickButton(m_buttonBoard, ButtonBox.Right1)
          .onTrue(m_gripper.extendCommand());

      new JoystickButton(m_buttonBoard, ButtonBox.Right3)
          .onTrue(m_gripper.retractCommand());
    }

    if (CLIMBERS_ENABLE) {
      new Trigger(() -> m_buttonBoard
          .getPOV() == Constants.OIConstants.ButtonBox.StickUp)
          .whileTrue(m_climbers.climbersUpCommand());
      new Trigger(() -> m_buttonBoard
          .getPOV() == Constants.OIConstants.ButtonBox.StickDown)
          .whileTrue(m_climbers.climbersDownCommand());

      Trigger safetySwitch = new Trigger(() -> m_buttonBoard.getRawButton(OIConstants.ButtonBox.SafetySwitch));
      Trigger lockClimbers = new Trigger(() -> m_buttonBoard.getRawButton(OIConstants.ButtonBox.EngineStart));
      
      safetySwitch.onTrue(new InstantCommand(() ->{ m_climbers.climberLockActive = true; }));

      lockClimbers.onTrue(new InstantCommand(() -> {
          m_climbers.climbersFullDown();
      }));
      
    }

    if (INTAKE_ARM_ENABLE) {
      new Trigger(() -> m_copilotController.getRightY() < -0.9 )
          .onTrue(m_intakeArm.setPositionCommand(IntakeConstants.kFloorPosition));
      
      new Trigger(() -> m_copilotController.getRightY() > 0.9)
          .onTrue(m_intakeArm.setPositionCommand(IntakeConstants.kElevatorPosition));

      new JoystickButton(m_copilotController, XboxController.Button.kRightStick.value)
          .onTrue(m_intakeArm.setPositionCommand(IntakeConstants.kTroughPosition));

      new Trigger(() -> m_intake.objectDetected())
          .onTrue(new Coral(m_intakeArm, m_intake));    

      new Trigger(()->m_buttonBoard.getRawButton(ButtonBox.RightKnobCW))
        .onTrue(m_intakeArm.changePositionCommand(IntakeConstants.kPositionIncrement));
      new Trigger(()->m_buttonBoard.getRawButton(ButtonBox.RightKnobCCW))
        .onTrue(m_intakeArm.changePositionCommand(-IntakeConstants.kPositionIncrement));
    }

    // m_reefController.getChangeTrigger()
    //   .onChange(new InstantCommand(()->{
    //         targetTagId = m_reefController.getAprilTagId();
    //         Pose2d tgt = m_reefController.getTargetPose2d();
    //         m_nav.m_dashboardField.getObject("dest").setPose(tgt);
    //         SmartDashboard.putString("ReefController", m_reefController.label());
    //       }
    //  ));

      

      if (INTAKE_ENABLE && INTAKE_ARM_ENABLE){
        // new JoystickButton(m_copilotController, XboxController.Button.kY.value)
        //   .onTrue(new LoadGripper(instance));

        new JoystickButton(m_copilotController, XboxController.Button.kA.value)
          .onTrue(new TroughScore(m_intakeArm, m_intake));
      }
  };

  public void configureTestControls() {
    JoystickButton testPlus = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Switch1Up);
    JoystickButton testMinus = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Switch1Down);


    if (INTAKE_ARM_ENABLE) {
      JoystickButton testIntakeArm = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Switch2Up);
      testIntakeArm.and(testPlus).whileTrue(m_intakeArm.testCommand(0.2));
      testIntakeArm.and(testMinus).whileTrue(m_intakeArm.testCommand(-0.2));    
    }

    if (INTAKE_ENABLE) {
      JoystickButton testLowerConveyor = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Switch3Down);
      testLowerConveyor.and(testPlus)    
          .onTrue(new InstantCommand(() -> m_intake.out()))
          .onFalse(new InstantCommand(() -> m_intake.setIntakeSpeed(0.0)));
      testLowerConveyor.and(testMinus) 
          .onTrue(new InstantCommand(() -> m_intake.in()))
          .onFalse(new InstantCommand(() -> m_intake.setIntakeSpeed(0.0)));

      // JoystickButton testUpperConveyor = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Switch3Up);
      // testUpperConveyor.and(testPlus)
      //     .onTrue(new InstantCommand(() -> m_intake.setUpperSpeed(Constants.IntakeConstants.upperLoadSpeed)))
      //     .onFalse(new InstantCommand(() -> m_intake.setUpperSpeed(0.0)));
      // testUpperConveyor.and(testMinus)
      //     .onTrue(new InstantCommand(() -> m_intake.setUpperSpeed(-Constants.IntakeConstants.upperLoadSpeed)))
      //     .onFalse(new InstantCommand(() -> m_intake.setUpperSpeed(0.0)));
    }

    if (ELEVATOR_ENABLE) {
      JoystickButton testElevator = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Switch2Down);
      testElevator.and(testPlus)
          .whileTrue(m_elevator.testMoveCommand(0.4));
      testElevator.and(testMinus)
          .whileTrue(m_elevator.testMoveCommand(-0.4));
    }

    if (CLIMBERS_ENABLE) {
      new Trigger(() -> m_buttonBoard
          .getPOV() == Constants.OIConstants.ButtonBox.StickLeft)
          .whileTrue(new InstantCommand(() -> m_climbers.test(0.10)));
      new Trigger(() -> m_buttonBoard
          .getPOV() == Constants.OIConstants.ButtonBox.StickRight)
          .whileTrue(new InstantCommand(() -> m_climbers.test(-0.10)));
      new Trigger(() -> m_buttonBoard
          .getPOV() == -1)
          .whileTrue(new InstantCommand(() -> m_climbers.test(0.0)));
    }

    if (GRIPPER_ENABLE) {
      JoystickButton testExtension = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Switch4Down);
      testExtension.and(testPlus)
          .whileTrue(m_gripper.testExtensionCommand(1.0));
      testExtension.and(testMinus)
          .whileTrue(m_gripper.testExtensionCommand(-1.0));
    }
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

  public void setLED(double value) {
    m_BlinkinLED.set(value);
  }

  /*
   * called once when is set to Red by the DriverStation
   */
  public void initRed() {
    m_robotDrive.setGyroAngleDeg(0.0);
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
    m_robotDrive.setGyroAngleDeg(180.0);
    m_autonomousChooser = AutonMenus.getBlue();
    SmartDashboard.putData("Auton Command", m_autonomousChooser);

    m_startPosChooser = StartPositions.getBlue();
    SmartDashboard.putData("Start Position", m_startPosChooser);
    m_startPosChooser.onChange(this::setStartPosition);
  }

  private void setStartPosition(Pose2d pose) {
    if (DriverStation.isDisabled()) {
      System.out.println("setStartPosition()" + pose.toString());
      m_robotDrive.setGyroAngleDeg(pose.getRotation().getDegrees());
      m_nav.resetOdometry(pose);
    }
  }

}
