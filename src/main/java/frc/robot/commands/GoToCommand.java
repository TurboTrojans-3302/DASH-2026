// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Navigation;

public class GoToCommand extends Command {

  protected final double dT = Robot.kDefaultPeriod;

  protected Pose2d m_dest;
  protected Transform2d m_delta;
  protected DriveTrain m_drive;
  protected TrapezoidProfile m_speedProfile;
  protected TrapezoidProfile m_angularSpeedProfile;
  protected boolean m_relativeFlag;
  protected Navigation m_nav;

  private static double defaultAccelLimit = AutoConstants.kMaxAccelerationMetersPerSecondSquared;
  private static double defaultSpeedLimit = AutoConstants.kMaxSpeedMetersPerSecond;
  private static double defaultAngularSpeedLimit = AutoConstants.kMaxAngularSpeedRadiansPerSecond;
  private static double defaultAngularAccelLimit = AutoConstants.kMaxAngularAccelRadiansPerSecondSquared;  
  private static double defaultDistanceTolerance = AutoConstants.kDistanceTolerance;
  private static double defaultHeadingTolerance =  AutoConstants.kHeadingTolerance;
  protected double accelLimit; 
  protected double speedLimit; 
  protected double angularSpeedLimit; 
  protected double angularAccelLimit;   
  protected double kDistanceTolerance; 
  protected double kHeadingTolerance; 

  public GoToCommand setLimits(double speedLimit, double accelLimit) {
    this.speedLimit = speedLimit;
    this.accelLimit = accelLimit;
    return this;
  } 

  public GoToCommand setTolerance(double distanceTolerance, double headingTolerance) {
    this.kDistanceTolerance = distanceTolerance;
    this.kHeadingTolerance = headingTolerance;
    return this;
  }

  protected GoToCommand(DriveTrain drive, Navigation nav) {
    accelLimit = defaultAccelLimit;
    speedLimit = defaultSpeedLimit;
    angularSpeedLimit = defaultAngularSpeedLimit;
    angularAccelLimit = defaultAngularAccelLimit;
    kDistanceTolerance = defaultDistanceTolerance;
    kHeadingTolerance = defaultHeadingTolerance;

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
    m_speedProfile = new TrapezoidProfile(new Constraints(speedLimit, accelLimit));
    m_angularSpeedProfile = new TrapezoidProfile(new Constraints(angularSpeedLimit, angularAccelLimit));
    Pose2d currPose2d = m_nav.getPose();

    if (m_relativeFlag) {
      m_dest = currPose2d.plus(m_delta);
    } else {
      m_delta = m_dest.minus(currPose2d);
    }

    // normalize the angles
    m_dest = new Pose2d(m_dest.getTranslation(), Rotation2d.fromRadians(MathUtil.angleModulus(m_dest.getRotation().getRadians())));
    m_delta = new Transform2d(m_delta.getTranslation(), Rotation2d.fromRadians(MathUtil.angleModulus(m_delta.getRotation().getRadians())));

    System.out.println("Starting go to: " + m_dest);
    m_nav.m_dashboardField.getObject("dest").setPose(m_dest);
    System.out.println("profile: " + m_speedProfile);
    System.out.println("angular profile: " + m_angularSpeedProfile);
  }

  protected double totalDistance() {
    return m_delta.getTranslation().getNorm();
  }

  protected Translation2d translation2dest() {
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

  protected double deltaHeadingRadians() {
    return MathUtil.angleModulus(destHeadingRadians() - m_nav.getHeading().getRadians());
  }

  protected double deltaHeadingDegrees() {
    return Math.toDegrees(deltaHeadingRadians());
  }

  protected double speedTowardTarget() {
    Translation2d botDirection = m_drive.getVelocityVector().rotateBy(m_nav.getHeading());
    Translation2d targetDirection = translation2dest();

    if (botDirection.getNorm() <= 1e-6) {
      return 0.0;
    } else if (targetDirection.getNorm() <= 1e-6) {
      return -m_drive.getSpeed();
    }

    double difference = targetDirection.getAngle().getRadians() - botDirection.getAngle().getRadians();
    return m_drive.getSpeed() * Math.cos(difference);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d toDest = translation2dest();
    double distanceToDest = toDest.getNorm();

    double traveledDistance = Math.max(0.0, totalDistance() - distanceToDest);
    State currentState = new State(traveledDistance, speedTowardTarget());
    State goalState = new State(totalDistance(), 0.0);

    double speed = m_speedProfile.calculate(dT, currentState, goalState).velocity;
    System.out.println("distance: " + distanceToDest + ", speed: " + speed);

    Translation2d unitTranslation;
    if (distanceToDest > 1e-6) {
      unitTranslation = toDest.div(distanceToDest);
    } else {
      unitTranslation = new Translation2d();
    }

    double angularDifference = deltaHeadingRadians();

    State angularCurrentState = new State(angularDifference, m_drive.getAngularVelocityRadPerSec());
    State angularGoalState = new State(0.0, 0.0);
    double angularVelocity = m_angularSpeedProfile.calculate(dT, angularCurrentState, angularGoalState).velocity;
    
    if(MathUtil.isNear(0.0, distanceToDest, kDistanceTolerance)){
      speed = 0.0;
    }

    m_drive.drive(unitTranslation.times(speed), angularVelocity, true);
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
    return MathUtil.isNear(0.0, distance(), kDistanceTolerance) &&
        MathUtil.isNear(destHeadingDegrees(), m_nav.getHeadingDegrees(),
                        kHeadingTolerance, 0.0, 360.0);
  }

  public static Sendable getSendable(){
    return new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("GoToCommand");
        builder.addDoubleProperty("speedLimit", () -> defaultSpeedLimit, (value) -> defaultSpeedLimit = value);
        builder.addDoubleProperty("accelLimit", () -> defaultAccelLimit, (value) -> defaultAccelLimit = value);
        builder.addDoubleProperty("angularSpeedLimit", () -> defaultAngularSpeedLimit, (value) -> defaultAngularSpeedLimit = value);
        builder.addDoubleProperty("angularAccelLimit", () -> defaultAngularAccelLimit, (value) -> defaultAngularAccelLimit = value);
        builder.addDoubleProperty("distanceTolerance", () -> defaultDistanceTolerance, (value) -> defaultDistanceTolerance = value);
        builder.addDoubleProperty("headingTolerance", () -> defaultHeadingTolerance, (value) -> defaultHeadingTolerance = value);
      }
    };
  }

}
