// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class Navigation extends SubsystemBase {
  private final String cameraName = "limelight";
  private DriveTrain m_drive;
  public Field2d m_dashboardField = new Field2d();
  protected SwerveDrivePoseEstimator m_poseEstimator;
  private static AprilTagFieldLayout m_aprilTagLayout;

  /** Creates a new Navigation. */
  public Navigation(DriveTrain drive) {
    this.m_drive = drive;

    m_poseEstimator = m_drive.getSwerveDrive().swerveDrivePoseEstimator;
    m_aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    LimelightHelpers.setPipelineIndex(cameraName, Constants.LimelightConstants.PipelineIdx.AprilTag);

    SmartDashboard.putData(m_dashboardField);
  }

  public void setAlliance(DriverStation.Alliance alliance) {
    if (alliance == DriverStation.Alliance.Red) {
      m_aprilTagLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
    } else {
      m_aprilTagLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
    PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
    //TODO is this necessary? how often is the estimate invalid?
    if (LimelightHelpers.validPoseEstimate(est) && 
        (Timer.getFPGATimestamp() - est.timestampSeconds) < 0.3) {
      m_poseEstimator.addVisionMeasurement(est.pose, est.timestampSeconds);
    }

    Pose2d pose = m_poseEstimator.getEstimatedPosition();
    LimelightHelpers.SetRobotOrientation(cameraName, pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    m_dashboardField.setRobotPose(pose);
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(Rotation2d.fromDegrees(m_drive.getGyroAngleDegrees()),
        m_drive.getSwerveModulePositions(),
        pose);
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


  /**
   * @return heading angle of the bot, according to the odometry
   */
  public Rotation2d getAngle() {
    return m_poseEstimator.getEstimatedPosition().getRotation();
  }

  /**
   * @return heading angle of the bot, according to the odometry, in degrees
   */
  public double getAngleDegrees() {
    return getAngle().getDegrees();
  }

  public double getDxToHubCenter() {
    Pose2d hubPose = Constants.FieldConstants.HubCenterPoint;
    Pose2d botPose = getPose();
    return hubPose.getTranslation().getDistance(botPose.getTranslation());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.addStringProperty("Pipeline", () -> LimelightHelpers.getCurrentPipelineType(cameraName), null);
      builder.addBooleanProperty("ValidTarget", () -> {return LimelightHelpers.getTV(cameraName);}, null);
      builder.addIntegerProperty("ApriltagFound", () -> {return (int) LimelightHelpers.getFiducialID(cameraName);} , null);
      builder.addStringProperty("DetectorFound", () -> {return LimelightHelpers.getDetectorClass(cameraName);}, null);
      builder.addStringProperty("ClassiferFound", () -> {return LimelightHelpers.getClassifierClass(cameraName);}, null);
      builder.addStringProperty("EstimatedPosition", ()->getPose().toString(), null);
    }
}
