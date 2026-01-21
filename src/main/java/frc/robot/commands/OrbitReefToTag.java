// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.frc2025.FieldConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Navigation;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OrbitReefToTag extends Command {
  private DriveSubsystem drive;
  private Navigation nav;
  private Rotation2d destAngle;
  private Pose2d reefCenterPose;
  private TrapezoidProfile profile;
  private double orbitVelocity = Constants.AutoConstants.kMaxSpeedMetersPerSecond;
  private double orbitAccel = Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared;

  /** Creates a new OrbitToAngle. */
  public OrbitReefToTag(DriveSubsystem drive, Navigation nav, int tagId) {
    addRequirements(drive);
    this.drive = drive;
    this.nav = nav;
    Pose2d tagPose = Navigation.getTagPose2d(tagId);
    this.destAngle = tagPose.getRotation();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //TODO change our center point to something from the 2026 game -- tower?
    reefCenterPose = new Pose2d((Robot.alliance == Alliance.Red ? FieldConstants.redVersion(FieldConstants.Reef.center) : FieldConstants.Reef.center), Rotation2d.kZero);
    profile = new TrapezoidProfile(new Constraints(orbitVelocity, orbitAccel));
    System.out.println("Start OrbitReefToTag: " + destAngle.toString());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d center = reefCenterPose.relativeTo(nav.getPose()).getTranslation();
    double currentSpeed = drive.getSpeed();
    double arcDistance = getArcDistance();
    double speed = profile.calculate(Robot.kDefaultPeriod,
                                     new State(0, currentSpeed * Math.signum(arcDistance)),
                                     new State(arcDistance, 0)).velocity;

    drive.orbitRobotFrame(speed, center);
  }

  private double getArcDistance(){
    Translation2d center = nav.getPose().relativeTo(reefCenterPose).getTranslation();
    Rotation2d currentAngle = center.getAngle();
    Double deltaRadians = MathUtil.angleModulus(destAngle.minus(currentAngle).getRadians());
    return deltaRadians * center.getNorm();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Finish OrbitReefToTag");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(getArcDistance()) < Constants.AutoConstants.kDistanceTolerance; 
  }
}