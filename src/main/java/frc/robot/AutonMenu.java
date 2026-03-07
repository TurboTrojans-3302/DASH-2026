package frc.robot;

import java.lang.reflect.Array;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DoNothing;
import frc.robot.commands.AutoAimRange;
import frc.robot.commands.AutoShoot;
public class AutonMenu {
    static RobotContainer bot = RobotContainer.getInstance();
    static ArrayList<Command> commands;
        public static SendableChooser<Boolean> toggleBuilder() {
            SendableChooser useBuilder = new SendableChooser<>();
            useBuilder.setDefaultOption("Use builder?", false);
            return useBuilder;
        }
    
        public static SendableChooser<Command> prebuiltAutos() {
            //tried and tested builds we know work
            SendableChooser autos = new SendableChooser<>();
            autos.setDefaultOption("Do Nothing", new DoNothing());
            return autos;
    }

    public static SendableChooser<Command> commandListBlue(){
      SendableChooser commandList = new SendableChooser<>();
      commandList.addOption("DoNothing", new DoNothing());
      commandList.addOption("AutoAimRange", new AutoAimRange(bot.m_robotDrive, bot.m_navigation, bot.m_shooter));
      commandList.addOption("AutoShoot", new AutoShoot(bot.m_robotDrive, bot.m_navigation, bot.m_shooter));

      return commandList;
    }   

    public static SendableChooser<Command> commandListRed(){
      SendableChooser commandList = new SendableChooser<>();
      commandList.addOption("DoNothing", new DoNothing());
      commandList.addOption("AutoAimRange", new AutoAimRange(bot.m_robotDrive, bot.m_navigation, bot.m_shooter));
      commandList.addOption("AutoShoot", new AutoShoot(bot.m_robotDrive, bot.m_navigation, bot.m_shooter));

      return commandList;
    }   

    public static Command builtAuton(){
    Command[] commandArray = (Command[]) commands.toArray();
    Command auton = new SequentialCommandGroup(commandArray);
    return auton;
    }
   

    

   


    

}
