// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class JostleShoot extends Command {
  Shooter m_shooter;
  Hopper m_hopper;
  private double startPosition;
  private double amplitude;
  private double period;
  private Timer feedTimer = new Timer();

  private final double forwardTime = 3.0;
  private final double backwardTime = 0.75;
  
  /** Creates a new JostleShoot. */
  public JostleShoot(Shooter shooter, Hopper hopper, double amplitude, double period) {
    m_shooter = shooter;
    m_hopper = hopper;
    this.amplitude = amplitude;
    this.period = period;
    addRequirements(m_hopper, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPosition = m_hopper.getPosition();
    m_shooter.feedForward();
    feedTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //move the hopper independently of the feeders
    double offset = amplitude * Math.sin(2 * Math.PI * (1.0 / period) * Timer.getFPGATimestamp());
    m_hopper.setPosition(startPosition + offset);

    // feed forward for 3.0s, then feed backward for 0.75s, then repeat
    if(feedTimer.get() < forwardTime){
      if(m_shooter.isReady()){
        m_shooter.feedForward();
      }else{
        m_shooter.stopFeeders();
      }
    } else if(feedTimer.get() < forwardTime + backwardTime){
      m_shooter.feedBackward();
    } else {
      feedTimer.restart();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hopper.setPosition(startPosition);
    m_shooter.stopFeeders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
