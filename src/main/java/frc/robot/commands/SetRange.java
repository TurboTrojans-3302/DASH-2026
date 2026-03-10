// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.rangeRPMtable;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetRange extends Command {
    private final Navigation navigation;
    private final Shooter shooter;

    /** Creates a new AutoAim. */
    public SetRange(Navigation navigation, Shooter shooter) {
        this.navigation = navigation;
        this.shooter = shooter;
        addRequirements(navigation, shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        setShooterSpeedForDistance();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return shooter.isReady();
    }


    public double getDXtoTarget(){
        return navigation.getDXtoTarget();
    }

    
    private void setShooterSpeedForDistance() {
        double rpm = rangeRPMtable.get(getDXtoTarget());
        shooter.setRPMsetpoint(rpm);
    }
}