// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.rangeRPMtable;

//todo move this into Shooter.java
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetRange extends Command {
  private final Shooter shooter;
  private DoubleSupplier distanceSupplier;
  private StringLogEntry log;

  public SetRange(Shooter shooter, Navigation nav) {
    this(shooter, () -> nav.getDXtoTarget());
  }

  public SetRange(Shooter shooter, double distance) {
    this(shooter, () -> distance);
  }
  
  public SetRange(Shooter shooter, DoubleSupplier distanceSupplier) {
    addRequirements(shooter);
    this.shooter = shooter;
    this.distanceSupplier = distanceSupplier;
    log = new StringLogEntry(Robot.getLog(), "SetRange");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double distance = distanceSupplier.getAsDouble();
    double rpm;
    if (rangeRPMtable.inRange(distance)) {
      rpm = rangeRPMtable.get(distance);
    } else {
      rpm = 1600.0;
    }
    shooter.setRPMsetpoint(rpm);
    log.append("distance = " + distance + " rpm = " + rpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        log.append("end(" + interrupted + ") rpm = " + shooter.getRPM());
  }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return shooter.isReady();
    }
}