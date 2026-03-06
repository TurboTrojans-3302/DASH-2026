package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DoNothing;

public class AutonMenu {
    RobotContainer bot = RobotContainer.getInstance();

    public static SendableChooser<Boolean> useBuilder() {
        SendableChooser useBuilder = new SendableChooser<>();
        useBuilder().setDefaultOption("Use builder?", false);
        return useBuilder;
    }

    public static SendableChooser<Command> prebuiltAutos() {
        //tried and tested builds we know work
        SendableChooser autos = new SendableChooser<>();
        autos.setDefaultOption("Do Nothing", new DoNothing());
        return autos;
    }


    

}
