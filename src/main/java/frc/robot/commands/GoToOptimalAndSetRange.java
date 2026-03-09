// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.rangeRPMtable;

/**
 * Extends GoToCommand to drive to the optimal shooting position (on the line
 * between the bot and the hub, at OPTIMAL range) and spin up the shooter.
 * Finishes when at position and shooter is at speed.
 */
public class GoToOptimalAndSetRange extends GoToCommand {

  private final Shooter shooter;

  public GoToOptimalAndSetRange(DriveTrain driveTrain, Navigation navigation, Shooter shooter) {
    super(driveTrain, navigation);
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    m_dest = m_nav.getOptimalShootPos();
    m_relativeFlag = false;
    super.initialize();

    setShooterSpeedForDistance();
  }

  @Override
  public void execute() {
    super.execute();
    setShooterSpeedForDistance();
  }

  @Override
  public boolean isFinished() {
    return super.isFinished() && shooter.isReady();
  }

  private void setShooterSpeedForDistance() {
    double rpm = rangeRPMtable.get(m_nav.getDXtoTarget());
    shooter.setRPMsetpoint(rpm);
  }
}
