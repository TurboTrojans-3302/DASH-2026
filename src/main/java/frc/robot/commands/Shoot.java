// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;

public class Shoot extends Command {
  Shooter m_shooter;
  public Shoot(Shooter shooter) {
    m_shooter = shooter;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    if(m_shooter.getRPM() < 2000){
      m_shooter.setRPMsetpoint(Constants.ShooterConstants.defaultShootRPM);
    }
  }

  @Override
  public void execute() {
    if(m_shooter.ready()){
      m_shooter.setFeederSpeed(Constants.ShooterConstants.feederSpeedDefault);
    } else {
      m_shooter.setFeederSpeed(0.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.setFeederSpeed(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
