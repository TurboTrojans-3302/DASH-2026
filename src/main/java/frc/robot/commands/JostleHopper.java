// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class JostleHopper extends Command {
  Hopper m_hopper;
  private double startPosition;
  private double amplitude;
  private double period;

  
  /** Creates a new JostleHopper. */
  public JostleHopper(Hopper hopper, double amplitude, double period) {
    m_hopper = hopper;
    this.amplitude = amplitude;
    this.period = period;
    addRequirements(m_hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPosition = m_hopper.getPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //move the hopper independently of the feeders
    double offset = amplitude * Math.sin(2 * Math.PI * (1.0 / period) * Timer.getFPGATimestamp());
    m_hopper.setPosition(startPosition + offset);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hopper.setPosition(startPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
