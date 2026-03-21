// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Navigation;

public class BetterGoToCommand extends Command {

  private static double kPX = AutoConstants.kPX;
  private static double kPY = AutoConstants.kPY;
  private static double kPTheta = AutoConstants.kPTheta;
    
  private final DriveTrain m_drive;
  private final Navigation m_nav;
  private Pose2d m_dest;
  private SwerveControllerCommand m_inner;

  protected final double dT = Robot.kDefaultPeriod;

  protected Transform2d m_delta;
  protected boolean m_relativeFlag;

  double accelLimit = AutoConstants.kMaxAccelerationMetersPerSecondSquared;
  double speedLimit = AutoConstants.kMaxSpeedMetersPerSecond;
  double angularSpeedLimit = AutoConstants.kMaxAngularSpeedRadiansPerSecond;
  double angularAccelLimit = AutoConstants.kMaxAngularAccelRadiansPerSecondSquared;  
  double kDistanceTolerance = AutoConstants.kDistanceTolerance;
  double kHeadingTolerance =  AutoConstants.kHeadingTolerance;

  public BetterGoToCommand setLimits(double speedLimit, double accelLimit) {
    this.speedLimit = speedLimit;
    this.accelLimit = accelLimit;
    return this;
  } 

  public BetterGoToCommand setTolerance(double distanceTolerance, double headingTolerance) {
    this.kDistanceTolerance = distanceTolerance;
    this.kHeadingTolerance = headingTolerance;
    return this;
  }

  protected BetterGoToCommand(DriveTrain drive, Navigation nav) {
    m_drive = drive;
    this.m_nav = nav;
    addRequirements(m_drive);
  }

  public BetterGoToCommand(DriveTrain drive, Navigation nav, Pose2d dest) {
    this(drive, nav);
    m_dest = dest;
    m_relativeFlag = false;
  }

  public static BetterGoToCommand absolute(DriveTrain drive, Navigation nav, Pose2d dest) {
    return new BetterGoToCommand(drive, nav, dest);
  }

  public static BetterGoToCommand absolute(DriveTrain drive, Navigation nav, double x, double y, double heading) {
    Pose2d dest = new Pose2d(x, y, Rotation2d.fromDegrees(heading));
    return new BetterGoToCommand(drive, nav, dest);
  }

  public static BetterGoToCommand relative(DriveTrain drive, Navigation nav, double x, double y, double theta) {
    Transform2d delta = new Transform2d(x, y, Rotation2d.fromDegrees(theta));
    return new BetterGoToCommand(drive, nav, delta);
  }
  
  public BetterGoToCommand(DriveTrain drive, Navigation nav, Transform2d delta) {
    this(drive, nav);
    m_delta = delta;
    m_relativeFlag = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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

    SwerveDriveKinematics kinematics = m_drive.getKinematics();
    PIDController xController = new PIDController(kPX, 0, 0);
    PIDController yController = new PIDController(kPY, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(kPTheta, 0, 0,
            new TrapezoidProfile.Constraints(angularSpeedLimit, angularAccelLimit));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    TrajectoryConfig config = new TrajectoryConfig(speedLimit, accelLimit)
            .setKinematics(kinematics);

    Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
            List.of(m_nav.getPose(), m_dest),
            config);

    m_nav.m_dashboardField.getObject("traj").setTrajectory(trajectory);

    m_inner = new SwerveControllerCommand(trajectory,
                                     m_drive::getPose,
                                     kinematics,
                                     xController, yController, thetaController,
                                     m_drive::setModuleStates,
                                     m_drive);
    m_inner.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_inner.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_inner.end(interrupted);
    m_drive.stop();
    System.out.println("End go to: " + m_nav.getPose());
  }

  @Override
  public boolean isFinished() {
    return m_inner.isFinished() || atTarget();
  }

  private boolean atTarget() {
    Transform2d delta = m_dest.minus(m_nav.getPose());
    double distance = delta.getTranslation().getNorm();
    double angle = MathUtil.angleModulus(delta.getRotation().getRadians());

    return Math.abs(distance) < kDistanceTolerance &&
            Math.abs(angle) < kHeadingTolerance;
  }

  public static Sendable getSendable(){
    return new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("BetterGoToCommand");
        builder.addDoubleProperty("kPX", () -> kPX, value -> kPX = value);
        builder.addDoubleProperty("kPY", () -> kPY, value -> kPY = value);
        builder.addDoubleProperty("kPTheta", () -> kPTheta, value -> kPTheta = value);
      }
    };
  }
}
