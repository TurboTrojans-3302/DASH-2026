// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cscore.VideoCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Navigation extends SubsystemBase {
  private final String cameraName = "limelight";
  private DriveTrain m_drive;
  private DXsensor m_dxSensor;
  public Field2d m_dashboardField = new Field2d();
  protected SwerveDrivePoseEstimator m_poseEstimator;
  private static AprilTagFieldLayout m_aprilTagLayout;
  private boolean mainCameraStreamSelected = true;
  private VideoCamera limeightCamera;
  private VideoCamera usbCamera;
  private VideoSink cameraServer;
  private Alliance alliance;

  /** Creates a new Navigation. */
  public Navigation(DriveTrain drive, DXsensor dxSensor) {
    this.m_drive = drive;
    this.m_dxSensor = dxSensor;

    m_poseEstimator = m_drive.getSwerveDrive().swerveDrivePoseEstimator;
    m_aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    m_aprilTagLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);


    LimelightHelpers.SetRobotOrientation(cameraName, m_drive.getGyroAngleDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.setPipelineIndex(cameraName, Constants.LimelightConstants.PipelineIdx.AprilTag);
    LimelightHelpers.setCameraPose_RobotSpace(cameraName,
                                              LimelightConstants.Offset.forward,
                                              LimelightConstants.Offset.side,
                                              LimelightConstants.Offset.up,
                                              LimelightConstants.Offset.pitch,
                                              LimelightConstants.Offset.yaw,
                                              LimelightConstants.Offset.roll);
    setIMUMode(1);

    // Publish the Limelight MJPEG stream so Elastic can display it as a camera widget
    limeightCamera = new HttpCamera(
        cameraName,
        "http://limelight.local:5800/stream.mjpg",
        HttpCameraKind.kMJPGStreamer);
    usbCamera = CameraServer.startAutomaticCapture();
    cameraServer = CameraServer.getServer();

    SmartDashboard.putData(m_dashboardField);
    SmartDashboard.putData("Nav Pose Heading", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", () -> getHeadingDegrees(), null);
      } 
    });

  }

  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
    if (alliance == Alliance.Red) {
      m_aprilTagLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
    } else {
    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

    // TODO is this necessary? how often is the estimate invalid?
    if (LimelightHelpers.validPoseEstimate(est) &&
        (Timer.getFPGATimestamp() - est.timestampSeconds) < 0.3) {
      m_poseEstimator.addVisionMeasurement(est.pose, est.timestampSeconds);
    }

    double yaw = alliance == Alliance.Blue ? m_drive.getGyroAngleDegrees() : m_drive.getGyroAngleDegrees() + 180.0;
    LimelightHelpers.SetRobotOrientation(cameraName, yaw, 0, 0, 0, 0, 0);

    m_dashboardField.setRobotPose(getPose());
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public static Pose2d getTagPose2d(int tagId) {
    return m_aprilTagLayout.getTagPose(tagId)
    .orElseThrow(() -> new RuntimeException("No AprilTag with ID " + tagId))
    .toPose2d();
  }

  //todo double check this method, not sure if the transform is correct
  public static Pose2d getPose2dInFrontOfTag(int tagId, double distance) {
    Transform2d delta = new Transform2d(distance, 0.0, Rotation2d.fromDegrees(180.0));
    Pose2d tagPose = getTagPose2d(tagId);
    return tagPose.plus(delta);
  }

  public void setPosition(Pose2d newPose) {
    m_drive.resetOdometry(newPose);
  }

  /**
   * @return heading angle of the bot, according to the odometry
   */
  public Rotation2d getHeading() {
    return m_poseEstimator.getEstimatedPosition().getRotation();
  }

  /**
   * @return heading angle of the bot, according to the odometry, in degrees
   */
  public double getHeadingDegrees() {
    return getHeading().getDegrees();
  }

  public Pose2d HubCenterPoint() {
    return alliance == Alliance.Blue ? FieldConstants.BlueHubCenterPoint : FieldConstants.RedHubCenterPoint;
  }

  public double getDxToHubCenter() {
    Pose2d hubPose = HubCenterPoint();
    Pose2d botPose = getPose();
    double odometryDistance = hubPose.getTranslation().getDistance(botPose.getTranslation());

    return odometryDistance;
  }

  /**
   * Get the distance to the target, fusing odometry with the laser sensor when available.
   * When the laser sensor is valid and close enough, and its reading agrees with odometry,
   * the laser reading is preferred.
   * @return distance to target in meters
   */
  public double getDXtoTarget() {
    double distance = getDxToHubCenter();
    
    if (m_dxSensor.isValid() && distance < 10.0) {
      double laserDx = m_dxSensor.getDistanceMeters() + 0.818; // odometryDX is center to center
                                                                // laser is front to front
      if (MathUtil.isNear(laserDx, distance, 0.5)) {
        distance = laserDx;
      }
    }
    return distance;
  }

  public void toggleCameraStream() {
    if(mainCameraStreamSelected) {
      cameraServer.setSource(usbCamera);
    } else {
      cameraServer.setSource(limeightCamera);
    }
    mainCameraStreamSelected = !mainCameraStreamSelected;
  }

  public Rotation2d getAbsBearingToTarget() {
    Translation2d delta = FieldConstants.HubCenterPoint.getTranslation().minus(getPose().getTranslation());
    return delta.getAngle();
  }

  public double getRelBearingToTargetDegrees() {
    return SwerveUtils.angleDeltaDeg(getHeadingDegrees(), getAbsBearingToTarget().getDegrees());
  } 

  /**
   * Get the optimal shooting position: a Pose2d on the line between the bot and the hub center,
   * at rangeRPMtable.OPTIMAL meters from the hub center, facing the hub.
   */
  public Pose2d getOptimalShootPos() {
    Translation2d hub = FieldConstants.HubCenterPoint.getTranslation();
    Translation2d bot = getPose().getTranslation();
    Translation2d hubToBot = bot.minus(hub);
    // Unit vector from hub toward bot
    Translation2d direction = hubToBot.div(hubToBot.getNorm());
    // Point OPTIMAL meters from hub along that line
    Translation2d optimalPoint = hub.plus(direction.times(rangeRPMtable.OPTIMAL));
    // Face the hub from that point
    Rotation2d heading = hub.minus(optimalPoint).getAngle();
    return new Pose2d(optimalPoint, heading);
  }
  
  public void setIMUMode(int mode) {
    LimelightHelpers.SetIMUMode(cameraName, mode);
  } 

  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.addStringProperty("Pipeline", () -> LimelightHelpers.getCurrentPipelineType(cameraName), null);
      builder.addBooleanProperty("ValidTarget", () -> {return LimelightHelpers.getTV(cameraName);}, null);
      builder.addIntegerProperty("ApriltagFound", () -> {return (int) LimelightHelpers.getFiducialID(cameraName);} , null);
      builder.addStringProperty("EstimatedPosition", ()->getPose().toString(), null);
      builder.addDoubleProperty("Target DX", ()->getDXtoTarget(), null);
      builder.addDoubleProperty("Target Angle", ()->getRelBearingToTargetDegrees(), null);
    }
}
