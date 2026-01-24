// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Shoot extends SequentialCommandGroup {
  /** Creates a new Shoot. */
  Shooter m_shooter;
  double shooterSpeed;
  double feederSpeed;
  public Shoot(Shooter shooter, double shooterSpeed, double feederSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_shooter = shooter;
    shooterSpeed = this.shooterSpeed;
    feederSpeed = this.feederSpeed;

    addCommands(new SpinUpShooter(m_shooter, shooterSpeed)
                .andThen(new toggleFeeder(m_shooter, feederSpeed))
                .andThen(new WaitCommand(4.0)) //TODO replace with command for detecting if there are still balls in the robot
                .andThen(new SpinUpShooter(m_shooter, 0.0))
                .andThen(new toggleFeeder(m_shooter, 0.0)));
  }
}
