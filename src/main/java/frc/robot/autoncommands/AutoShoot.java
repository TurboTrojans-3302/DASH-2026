// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoncommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimAtHub;
import frc.robot.commands.GoToCommand;
import frc.robot.commands.JostleShoot;
import frc.robot.commands.SetRange;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hopper;
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
  private Hopper m_hopper;
  public AutoShoot(DriveTrain driveTrain, Shooter shooter, Navigation navigation, Hopper hopper) {
    m_driveTrain = driveTrain;
    m_shooter = shooter;
    m_navigation = navigation;
    m_hopper = hopper;

    addCommands(

      GoToCommand.relative(m_driveTrain, m_navigation, 1.0, 0.0, 0.0)
                            .setLimits(4.0, 8.0)
                            .withTimeout(5.0),
      
      new SetRange(m_shooter, m_navigation).alongWith(
        new AimAtHub(m_navigation, m_driveTrain),
        m_hopper.setPositionCommand(Hopper.STARTPOSITION)
      ),
      
      new JostleShoot(m_shooter, m_hopper, 10, 2.0)
          .withTimeout(14)); 
  }
}
