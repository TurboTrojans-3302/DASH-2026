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
public class AutoShootFromCenter extends SequentialCommandGroup {
  /** Creates a new AutoShootFromCenter. */
  public AutoShootFromCenter(DriveTrain drivetrain, Shooter shooter, Navigation nav, Hopper hopper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      GoToCommand.relative(drivetrain, nav, 1, 0, 0).alongWith(
        hopper.setPositionCommand(Hopper.STARTPOSITION)
      ).withTimeout(3.0),
      new AimAtHub(nav, drivetrain).withTimeout(3.0),
      new SetRange(shooter, nav).withTimeout(5.0),
      new JostleShoot(shooter, hopper, 10, 2.0)
          .withTimeout(14)
    );
  }
}
