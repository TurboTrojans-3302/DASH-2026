// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoncommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GoToCommand;
import frc.robot.commands.JostleShoot;
import frc.robot.commands.SetRange;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Shooter;

public class AutoSideStartMoveAndShootNoNav extends SequentialCommandGroup {
  /** Creates a new AutoSideStartMoveAndShootNoNav. */
  public AutoSideStartMoveAndShootNoNav(DriveTrain driveTrain, Navigation navigation, Shooter shooter, Hopper hopper, double dx, double angle) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(GoToCommand.relative(driveTrain, navigation, dx, 0.0, angle)
                              .alongWith(
                                new SetRange(shooter, dx),
                                hopper.setPositionCommand(Hopper.STARTPOSITION)),
                new JostleShoot(shooter, hopper, 10, 2.0)
                              .withTimeout(14)
    );
  }
}
