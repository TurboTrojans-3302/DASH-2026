// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Navigation;
import frc.utils.SwerveUtils;

public class GoToCommand extends Command {

  private final double dT = Robot.kDefaultPeriod;

  protected Pose2d m_dest;
  protected Transform2d m_delta;
  protected DriveTrain m_drive;
  private TrapezoidProfile m_trapezoid;
  protected boolean m_relativeFlag;
  protected Navigation m_nav;

  static double speedLimit = AutoConstants.kMaxSpeedMetersPerSecond;
  static double accelLimit = AutoConstants.kMaxAccelerationMetersPerSecondSquared;
  static double kDistanceTolerance = Constants.AutoConstants.kDistanceTolerance;
  static double kHeadingTolerance = Constants.AutoConstants.kHeadingTolerance;

  protected GoToCommand(DriveTrain drive, Navigation nav) {
    m_drive = drive;
    this.m_nav = nav;
    addRequirements(m_drive);
  }

  public GoToCommand(DriveTrain drive, Navigation nav, Pose2d dest) {
    this(drive, nav);
    m_dest = dest;
    m_relativeFlag = false;
  }

  public static GoToCommand absolute(DriveTrain drive, Navigation nav, Pose2d dest) {
    return new GoToCommand(drive, nav, dest);
  }

  public static GoToCommand absolute(DriveTrain drive, Navigation nav, double x, double y, double heading) {
    Pose2d dest = new Pose2d(x, y, Rotation2d.fromDegrees(heading));
    return new GoToCommand(drive, nav, dest);
  }

  public static GoToCommand relative(DriveTrain drive, Navigation nav, double x, double y, double theta) {
    Transform2d delta = new Transform2d(x, y, Rotation2d.fromDegrees(theta));
    return new GoToCommand(drive, nav, delta);
  }
  
  public GoToCommand(DriveTrain drive, Navigation nav, Transform2d delta) {
    this(drive, nav);
    m_delta = delta;
    m_relativeFlag = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_trapezoid = new TrapezoidProfile(new Constraints(speedLimit, accelLimit));

    if (m_relativeFlag) {
      Pose2d currPose2d = m_nav.getPose();
      m_dest = currPose2d.plus(m_delta);
    }
    System.out.println("Starting go to: " + m_dest);
    m_nav.m_dashboardField.getObject("dest").setPose(m_dest);
  }

  private Translation2d translation2dest() {
    return m_dest.getTranslation().minus(m_nav.getPose().getTranslation());
  }

  protected double distance() {
    return translation2dest().getNorm();
  }

  public double destHeadingDegrees() {
    return m_dest.getRotation().getDegrees();
  }

  public double destHeadingRadians() {
    return m_dest.getRotation().getRadians();
  }

  private double deltaHeading() {
    return SwerveUtils.angleDeltaDeg(m_nav.getAngleDegrees(), destHeadingDegrees());
  }

  private double speedTowardTarget() {
    Translation2d botDirection = m_drive.getVelocityVector().rotateBy(m_nav.getAngle());
    Translation2d targetDirection = translation2dest();

    if (botDirection.getNorm() <= 1e-6) {
      return 0.0;
    } else if (targetDirection.getNorm() <= 1e-6) {
      return -m_drive.getSpeed();
    }

    Double difference = targetDirection.getAngle().getRadians() - botDirection.getAngle().getRadians();
    return m_drive.getSpeed() * Math.cos(difference);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    State currentState = new State(0.0, speedTowardTarget());
    State goalState = new State(distance(), 0.0);

    double speed = m_trapezoid.calculate(dT, currentState, goalState).velocity;

    Translation2d toDest = translation2dest();
    if(toDest.getNorm() < 1e-6){
      return; // do nothing
    }

    Translation2d unitTranslation = toDest.div(toDest.getNorm());

    m_drive.driveHeading(unitTranslation.times(speed), destHeadingRadians());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
    System.out.println("End go to: " + m_nav.getPose());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distance() < kDistanceTolerance &&
        Math.abs(deltaHeading()) < kHeadingTolerance;
  }

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("speedLimit", () -> speedLimit, (x) -> speedLimit = x );
    builder.addDoubleProperty("accelLimit", () -> accelLimit, (x) -> accelLimit = x );
    builder.addDoubleProperty("kDistanceTolerance", () -> kDistanceTolerance, (x) -> kDistanceTolerance = x );
    builder.addDoubleProperty("kHeadingTolerance", () -> kHeadingTolerance, (x) -> kHeadingTolerance = x );
    builder.addDoubleProperty("dest X", ()->m_dest.getX(), (x)->{m_dest = new Pose2d(x, m_dest.getY(), m_dest.getRotation()); });
    builder.addDoubleProperty("dest Y", ()->m_dest.getY(), (y)->{m_dest = new Pose2d(m_dest.getX(), y, m_dest.getRotation()); });
    builder.addDoubleProperty("dest ϴ", ()->m_dest.getRotation().getDegrees(), (t)->{m_dest = new Pose2d(m_dest.getX(), m_dest.getY(), Rotation2d.fromDegrees(t)); });
  }
}
