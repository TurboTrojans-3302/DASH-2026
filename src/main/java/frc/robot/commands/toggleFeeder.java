// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class toggleFeeder extends InstantCommand {
  /** Creates a new SpinUpShooter. */
  Shooter m_shooter;
  double feederSpeed;

  public toggleFeeder(Shooter shooter, double feederSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    feederSpeed = this.feederSpeed;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setFeederSpeed(feederSpeed);
  }
}
