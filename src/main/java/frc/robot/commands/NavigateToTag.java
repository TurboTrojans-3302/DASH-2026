// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Navigation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NavigateToTag extends SequentialCommandGroup {
  /** Creates a new NavigateToTag. */
  public NavigateToTag(DriveSubsystem drive, Navigation nav, IntSupplier getTagId) {
    int tagid = getTagId.getAsInt();

    if(tagid < 1) {
      // do nothing?
    }else{
        System.out.println("Start NavigateToTag " + tagid);

        addCommands(new DriveCloseToReef(drive, nav),
                    new OrbitReefToTag(drive, nav, tagid),
                    new GoToCommand(drive, nav, Navigation.getPose2dInFrontOfTag(tagid, 0.05))
                    );
    }
  }
}