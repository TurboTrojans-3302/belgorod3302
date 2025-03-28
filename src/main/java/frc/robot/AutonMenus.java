// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutonRemoveAlgae;
import frc.robot.commands.DoNothing;
import frc.robot.commands.DriveToAprilTag;
import frc.robot.commands.GoToCommand;
import frc.robot.commands.TurnToAprilTag;
import frc.robot.commands.Autonomous.AutonCoralTrough;
import frc.robot.subsystems.Navigation;

/** Add your docs here. */
public class AutonMenus {

    public static SendableChooser<Command> getBlue(){
        RobotContainer bot = RobotContainer.getInstance();
    
        SendableChooser<Command> blueCommands = new SendableChooser<Command>();
        blueCommands.setDefaultOption("DoNothing (blue)", new DoNothing());
        blueCommands.addOption("TroughScore 17 far left", new AutonCoralTrough(bot, 17));
        blueCommands.addOption("TroughScore 18 far center", new AutonCoralTrough(bot, 18));
        blueCommands.addOption("TroughScore 19 far right", new AutonCoralTrough(bot, 19));
        blueCommands.addOption("TroughScore 20 near right", new AutonCoralTrough(bot, 20));
        blueCommands.addOption("TroughScore 21 near center", new AutonCoralTrough(bot, 21));
        blueCommands.addOption("TroughScore 22 near left", new AutonCoralTrough(bot, 22));

        blueCommands.addOption("RemoveAlgae 17 far left", new AutonRemoveAlgae(bot, 17));
        blueCommands.addOption("RemoveAlgae 18 far center", new AutonRemoveAlgae(bot, 18));
        blueCommands.addOption("RemoveAlgae 19 far right", new AutonRemoveAlgae(bot, 19));
        blueCommands.addOption("RemoveAlgae 20 near right", new AutonRemoveAlgae(bot, 20));
        blueCommands.addOption("RemoveAlgae 21 near center", new AutonRemoveAlgae(bot, 21));
        blueCommands.addOption("RemoveAlgae 22 near left", new AutonRemoveAlgae(bot, 22));

        blueCommands.addOption("go fwd 1m", GoToCommand.relative(bot.m_robotDrive, bot.m_nav, 1.0, 0, 0));
        blueCommands.addOption("go fwd 2m", GoToCommand.relative(bot.m_robotDrive, bot.m_nav, 2.0, 0, 0));
        
        return blueCommands;
    }

    public static SendableChooser<Command> getRed(){
        RobotContainer bot = RobotContainer.getInstance();

        SendableChooser<Command> redCommands = new SendableChooser<Command>();
        redCommands.setDefaultOption("Do Nothing (red)", new DoNothing());
        redCommands.addOption("TroughScore 6 far right", new AutonCoralTrough(bot, 6));
        redCommands.addOption("TroughScore 7 far center", new AutonCoralTrough(bot, 7));
        redCommands.addOption("TroughScore 8 far left", new AutonCoralTrough(bot, 8));
        redCommands.addOption("TroughScore 9 near left", new AutonCoralTrough(bot, 9));
        redCommands.addOption("TroughScore 10 front", new AutonCoralTrough(bot, 10));
        redCommands.addOption("TroughScore 11 near right", new AutonCoralTrough(bot, 11));

        redCommands.setDefaultOption("Do Nothing (red)", new DoNothing());
        redCommands.addOption("RemoveAlgae 6 far right", new AutonRemoveAlgae(bot, 6));
        redCommands.addOption("RemoveAlgae 7 far center", new AutonRemoveAlgae(bot, 7));
        redCommands.addOption("RemoveAlgae 8 far left", new AutonRemoveAlgae(bot, 8));
        redCommands.addOption("RemoveAlgae 9 near left", new AutonRemoveAlgae(bot, 9));
        redCommands.addOption("RemoveAlgae 10 front", new AutonRemoveAlgae(bot, 10));
        redCommands.addOption("RemoveAlgae 11 near right", new AutonRemoveAlgae(bot, 11));

        redCommands.addOption("go fwd 1m", GoToCommand.relative(bot.m_robotDrive, bot.m_nav, 1.0, 0, 0));
        redCommands.addOption("go fwd 2m", GoToCommand.relative(bot.m_robotDrive, bot.m_nav, 2.0, 0, 0));
        
        return redCommands;
    }
}
