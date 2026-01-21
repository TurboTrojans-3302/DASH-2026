// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.frc2025.FieldConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Navigation;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCloseToReef extends GoAlmostTo {
  /** Creates a new DriveCloseToReef. */
  public DriveCloseToReef(DriveSubsystem drive, Navigation nav) {
    super(drive, nav, null, 1.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Translation2d reef = (Robot.alliance == Alliance.Red ? FieldConstants.redVersion(FieldConstants.Reef.center) : FieldConstants.Reef.center);
    m_dest = new Pose2d(reef, Rotation2d.kZero);
    super.initialize();
  }
}