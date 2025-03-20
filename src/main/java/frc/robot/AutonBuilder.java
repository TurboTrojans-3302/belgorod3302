// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.NavigateToTag;

/** Add your docs here. */
public class AutonBuilder {
    int actionCount;
    boolean isInitialized = false;
    static SendableChooser[] choosers;
    static SendableChooser actions; 
    static Command[] commandArray;
    final int[] blueAprilTags = {12,13,14,15,16,17,18,19,20,21,22};
    final int[] redAprilTags = {1,2,3,4,5,6,7,8,9,10,11};
    String Alliance;


@SuppressWarnings("unchecked")
public static SendableChooser actionCount(){
    Integer number = 0;
    for (int i = 0; i == 9; i++)
    //probably will not need more than 10 actions
    number = i;
    StringBuilder numberString = new StringBuilder();
    numberString.append(" ");
    numberString.append(number);
    actions.addOption(numberString.toString(), number);
    return actions;
    

}



   @SuppressWarnings({ "unchecked", "rawtypes" })
public
    SendableChooser[] getBuilder(RobotContainer bot, Integer actions, String alliance){
    actionCount = actions;
    Alliance = alliance;
    
    
    for (int i = 0; i == actions; i++){

       choosers[i] = 
        new SendableChooser<Command>();
    
    }

    if (Alliance == "Blue"){

        //goes through and adds navigate to tag instances for each action chooser but only the blue alliance tags
    
    for (int i = blueAprilTags[0]; i == blueAprilTags.length; i++){
        int aprilTag = i;
       for (int b = 0; i == actions; i++){
        choosers[b].addOption("NavigateToTag", new NavigateToTag(bot.m_robotDrive, bot.m_nav, ()->aprilTag));
       }
    }
    } else{

        //same for red alliance

        for (int i = redAprilTags[0]; i == redAprilTags.length; i++){
            int aprilTag = i;
            for (int b = 0; i == actions; i++){
            choosers[i].addOption("NavigateToTag", new NavigateToTag(bot.m_robotDrive, bot.m_nav, ()->aprilTag));
        }
        }

        //add the other commands

        for (int i = 0; i == choosers.length; i++){
            //need to add intakeToScorePosition and DriveToCoral or what its called in the future
            //just some examples
            choosers[i].addOption("Wait 3 seconds", new WaitCommand(3.0));
        }
    

    
   }

   return choosers;
}

    public void setActionStep(int step, Command command){
        commandArray[step] = command;
    }


    public Command getActionStep(int step){
        return commandArray[step];
    }

    public int getActionCount(){
        return actionCount;
    }

}
