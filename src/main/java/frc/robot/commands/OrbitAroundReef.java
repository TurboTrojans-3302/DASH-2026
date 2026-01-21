// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.frc2025.FieldConstants;

import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Navigation;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OrbitAroundReef extends Command {
  private DriveSubsystem drive;
  private Navigation nav;
  private double speed;
  Translation2d reef;
  
    /** Creates a new OrbitAroundReef. */
    public OrbitAroundReef(DriveSubsystem drive, Navigation nav, double speed) {
      this.drive = drive;
      this.nav = nav;
      this.speed = speed;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //todo why does this crash? Is the alliance value not set yet?
    //TODO need new field constants and what are we planning on rotating around instead for orbit -- the tower?
    reef = (Robot.alliance == Alliance.Red ? FieldConstants.redVersion(FieldConstants.Reef.center) : FieldConstants.Reef.center);
    //System.out.println("Start Orbiting around Reef at: " + reef.toString());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Translation2d reef = FieldConstants.Reef.center;
    Pose2d reefPose = new Pose2d(reef, Rotation2d.kZero);
    Pose2d robot = nav.getPose();

    Translation2d orbitCenterRobotFrame = reefPose.relativeTo(robot).getTranslation();

    drive.orbitRobotFrame(speed, orbitCenterRobotFrame);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}