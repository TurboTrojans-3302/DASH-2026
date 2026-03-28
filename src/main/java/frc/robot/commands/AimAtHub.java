// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Navigation;

/*
 * Because the shooter is on the back of the robot,
 * we want to drive backwards towards the hub.
 * This command will use the nav subsystem to get the angle
 * to the target and then rotate in that direction until
 * we are within 1 degree of the target.
 */
public class AimAtHub extends Command {
  Navigation nav;
  DriveTrain drive;
  Rotation2d targetAngle;
  StringLogEntry log;

  /** Creates a new AimAtHub. */
  public AimAtHub(Navigation nav, DriveTrain drive) {
    this.nav = nav;
    this.drive = drive;
    addRequirements(nav, drive);
    log = new StringLogEntry(Robot.getLog(), this.getName());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetAngle = nav.getAbsBearingToTarget();
    log.append("aiming at: " + targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.driveHeading(Translation2d.kZero, targetAngle.plus(Rotation2d.k180deg));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    log.append("end(" +  interrupted + ") target: " + targetAngle.getDegrees() + " actual: " + nav.getHeadingDegrees());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return MathUtil.isNear(targetAngle.getDegrees(), nav.getHeadingDegrees(), 1.0, -180, 180);
  }
}
