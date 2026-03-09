// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoncommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GoToOptimalAndSetRange;
import frc.robot.commands.SetRangeAndAim;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShoot extends SequentialCommandGroup {
  /** Creates a new AutoShoot. */
  private DriveTrain m_driveTrain;
  private Shooter m_shooter;
  private Navigation m_navigation;
  public AutoShoot(DriveTrain driveTrain, Shooter shooter, Navigation navigation) {
    m_driveTrain = driveTrain;
    m_shooter = shooter;
    m_navigation = navigation;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new GoToOptimalAndSetRange(driveTrain, navigation, shooter),
                new SetRangeAndAim(m_driveTrain, m_navigation, m_shooter),
                m_shooter.shootCommand().withTimeout(10)); 
  }
}
