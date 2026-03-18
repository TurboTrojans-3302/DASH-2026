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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Navigation;
import frc.utils.SwerveUtils;

public class GoToCommand extends Command {

  private static double globalSpeedScale = 1.0;
  private static double globalToleranceScale = 1.0;
  protected final double dT = Robot.kDefaultPeriod;

  protected Pose2d m_dest;
  protected Transform2d m_delta;
  protected DriveTrain m_drive;
  protected TrapezoidProfile m_speedProfile;
  protected TrapezoidProfile m_angularSpeedProfile;
  protected boolean m_relativeFlag;
  protected Navigation m_nav;
  protected double m_totalDistance;  // total distance to goal at initialize()

  double accelLimit = AutoConstants.kMaxAccelerationMetersPerSecondSquared;
  double speedLimit = AutoConstants.kMaxSpeedMetersPerSecond;
  double angularSpeedLimit = AutoConstants.kMaxAngularSpeedRadiansPerSecond;
  double angularAccelLimit = AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared;  
  double kDistanceTolerance = AutoConstants.kDistanceTolerance;
  double kHeadingTolerance =  AutoConstants.kHeadingTolerance;


  public static void setGlobalSpeedScale(double scale) {
    globalSpeedScale = MathUtil.clamp(scale, 0.0, 1.0);
  }

  public static void setGlobalToleranceScale(double scale) {
    globalToleranceScale = MathUtil.clamp(scale, 0.0, 10.0);
  }

  public static double getGlobalToleranceScale() {
    return globalToleranceScale;
  }

  public GoToCommand setLimits(double speedLimit, double accelLimit) {
    this.speedLimit = speedLimit;
    this.accelLimit = accelLimit;
    return this;
  } 

  public GoToCommand setTolerance(double distanceTolerance, double headingTolerance) {
    this.kDistanceTolerance = distanceTolerance;
    this.kHeadingTolerance = headingTolerance;
    this.setName(getName());
    return this;
  }



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
    m_speedProfile = new TrapezoidProfile(new Constraints(globalSpeedScale * speedLimit, globalSpeedScale * accelLimit));
    m_angularSpeedProfile = new TrapezoidProfile(new Constraints(globalSpeedScale * angularSpeedLimit, globalSpeedScale * angularAccelLimit));

    if (m_relativeFlag) {
      Pose2d currPose2d = m_nav.getPose();
      m_dest = currPose2d.plus(m_delta);
    }
    m_totalDistance = distance();
    System.out.println("Starting go to: " + m_dest);
    m_nav.m_dashboardField.getObject("dest").setPose(m_dest);
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

  protected double deltaHeading() {
    return SwerveUtils.angleDeltaDeg(m_nav.getAngleDegrees(), destHeadingDegrees());
  }

  protected double speedTowardTarget() {
    Translation2d botDirection = m_drive.getVelocityVector().rotateBy(m_nav.getAngle());
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

    double traveledDistance = Math.max(0.0, m_totalDistance - distanceToDest);
    State currentState = new State(traveledDistance, speedTowardTarget());
    State goalState = new State(m_totalDistance, 0.0);

    double speed = m_speedProfile.calculate(dT, currentState, goalState).velocity;

    Translation2d unitTranslation;
    if (distanceToDest > 1e-6) {
      unitTranslation = toDest.div(distanceToDest);
    } else {
      unitTranslation = new Translation2d();
    }

    double currentHeading = m_drive.getHeading().getRadians();
    double goalHeading = destHeadingRadians();
    double angleError = MathUtil.angleModulus(goalHeading - currentHeading);
    State angularCurrentState = new State(angleError, -m_drive.getAngularVelocityRadPerSec());
    State angularGoalState = new State(0.0, 0.0);
    double profiledErrorVelocity = m_angularSpeedProfile.calculate(dT, angularCurrentState, angularGoalState).velocity;
    double rotation = -profiledErrorVelocity;
    
    if(MathUtil.isNear(0.0, distanceToDest, kDistanceTolerance * globalToleranceScale)){
      speed = 0.0;
    }

    m_drive.drive(unitTranslation.times(speed), rotation, true);
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
    return MathUtil.isNear(0.0, distance(), kDistanceTolerance * globalToleranceScale) &&
        MathUtil.isNear(destHeadingDegrees(), m_nav.getAngleDegrees(),
                        kHeadingTolerance * globalToleranceScale, 0.0, 360.0);
  }

public static double getGlobalSpeedScale() {
    return globalSpeedScale;
}

}
