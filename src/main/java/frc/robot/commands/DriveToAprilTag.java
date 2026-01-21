// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Navigation;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToAprilTag extends Command {
  public static final String cameraName = "limelight";

  DriveSubsystem m_drive;
  int m_targetTag;
  boolean targetFound = false;
  RawFiducial detectedTarget;

  public static final double TurnTolerance = 1.5;
  public static final double TurnFactor = 1.0;
  public static final double ForwardSpeed = 0.2;
  public static final double TargetDistance = 0.09; // meters

  private Navigation m_nav;

  /** Creates a new TurnToAprilTag. */
  public DriveToAprilTag(DriveSubsystem drive, Navigation nav, int apriltag) {
    m_drive = drive;
    this.m_nav = nav;
    m_targetTag = apriltag;

    addRequirements(m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPriorityTagID(cameraName, m_targetTag);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Double heading = m_drive.getGyroAngleDegrees();

    if (isTargetFound()) {
      Double errorAngle = getAngleToTarget();
      heading = heading - (errorAngle * TurnFactor);

    }

    m_drive.driveHeadingRobot(new Translation2d(ForwardSpeed, 0.0), heading);
    SmartDashboard.putNumber("Target Angle", getAngleToTarget());
    SmartDashboard.putNumber("Commanded heading", heading);
    SmartDashboard.putNumber("Target Distance", getDistance());
    SmartDashboard.putBoolean("Target Found", isTargetFound());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_nav.getDxToObjectMeters() < TargetDistance;
  }

  boolean isTargetFound() {
    return LimelightHelpers.getTV(cameraName);
  }

  Double getAngleToTarget() {
    return LimelightHelpers.getTX(cameraName);
  }

  public double getDistance() {
    Pose3d targetPose = LimelightHelpers.getTargetPose3d_CameraSpace(cameraName);
    double distance = targetPose.getTranslation().getNorm();
    System.out.println("Distance:" + distance);

    return distance;
  }
}