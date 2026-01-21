// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Navigation;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoAlmostTo extends GoToCommand {
  private double m_distanceAway;

  /** Creates a new GoAlmostTo. */
  public GoAlmostTo(DriveSubsystem drive, Navigation nav, Pose2d target, double distanceAway) {
    super(drive, nav, target);
    this.m_distanceAway = distanceAway;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d bot = m_nav.getPose();
    Translation2d translation = m_dest.getTranslation().minus(bot.getTranslation());
    Rotation2d angle = translation.getAngle();
    m_dest = new Pose2d(m_dest.getTranslation(), angle);
    super.initialize();
  }

  @Override
  protected double distance(){
    return Math.max(super.distance() - m_distanceAway, 0.0);
  }
}