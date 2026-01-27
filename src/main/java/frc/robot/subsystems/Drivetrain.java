// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.Navx; // TODO find the library for the gyro, it is the only cause of errors in this file

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CanIds;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystemBase;

public class Drivetrain extends DriveSubsystemBase {
  private double maxSpeedLimit = DriveConstants.kMaxSpeedMetersPerSecond; // m/s
  private double maxRotationLimit = DriveConstants.kMaxAngularSpeed;

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = MAXSwerveModule.getInstance(
      CanIds.kFrontLeftDrivingCanId,
      CanIds.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = MAXSwerveModule.getInstance(
      CanIds.kFrontRightDrivingCanId,
      CanIds.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = MAXSwerveModule.getInstance(
      CanIds.kRearLeftDrivingCanId,
      CanIds.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = MAXSwerveModule.getInstance(
      CanIds.kRearRightDrivingCanId,
      CanIds.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final Navx m_gyro = new Navx(99);
  private double m_gyroOffsetDeg = 0.0;

  public PIDController headingPidController;
  private ChassisSpeeds CommandedSpeeds = new ChassisSpeeds();

  /** Creates a new DriveSubsystem. */
  public Drivetrain() {
    headingPidController = new PIDController(DriveConstants.headingP,
        DriveConstants.headingI,
        DriveConstants.headingD);
    headingPidController.enableContinuousInput(0.0, 360.0);
    headingPidController.setTolerance(2.0);

    SmartDashboard.putData("headingPIDcontroller", headingPidController);
    //SmartDashboard.putData("IMU", m_gyro.toString()); // TODO fix this line if the gyro library is found

    // todo add the swerve drive to the dashboard
    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> m_frontLeft.getSteerAngle(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> m_frontLeft.getDriveVelocity(), null);

        builder.addDoubleProperty("Front Right Angle", () -> m_frontRight.getSteerAngle(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> m_frontRight.getDriveVelocity(), null);

        builder.addDoubleProperty("Back Left Angle", () -> m_rearLeft.getSteerAngle(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> m_rearLeft.getDriveVelocity(), null);

        builder.addDoubleProperty("Back Right Angle", () -> m_rearRight.getSteerAngle(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> m_rearRight.getDriveVelocity(), null);

        builder.addDoubleProperty("Robot Angle", () -> getGyroAngleRadians(), null);
      }
    });
    SmartDashboard.putData("Left Front Module", m_frontLeft);
    SmartDashboard.putData("Right Front Module", m_frontRight);
    SmartDashboard.putData("Left Rear Module", m_rearLeft);
    SmartDashboard.putData("Right Rear Module", m_rearRight);

  }

  @Override
  public void periodic() {

  }

  public double turnToHeadingDegrees(double targetHeading) {
    double currentHeading = getGyroAngleDegrees();
    double rotation = headingPidController.calculate(currentHeading, targetHeading);
    return rotation;
  }

  public void driveRobotOriented(Double xSpeed, Double ySpeed, Double rot) {

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * maxSpeedLimit;
    double ySpeedDelivered = ySpeed * maxSpeedLimit;
    double rotDelivered = rot * maxRotationLimit;

    ChassisSpeeds speeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

    drive(speeds);
  }

  public void driveFieldOriented(Double xSpeed, Double ySpeed, Double rot) {

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * maxSpeedLimit;
    double ySpeedDelivered = ySpeed * maxSpeedLimit;
    double rotDelivered = rot * maxRotationLimit;

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
        Rotation2d.fromDegrees(getGyroAngleDegrees()));

    drive(speeds);
  }

  public void drive(ChassisSpeeds speeds, Translation2d centerOfRotation) {
    CommandedSpeeds = speeds;
    var swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(speeds, centerOfRotation);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeedLimit);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

    //if(Robot.isSimulation()) {
      //double turnRate = speeds.omegaRadiansPerSecond;
     // m_gyroSim.setGyroRateX(-Math.toDegrees(turnRate));
      //m_gyroSim.setGyroAngleX(-Math.toDegrees(turnRate * Robot.kDefaultPeriod) + m_gyro.getAngle());
   // }
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void testSetAll(double voltage, double angle) {
    // m_frontLeft.testSet(voltage, angle);
    //  m_frontRight.testSet(voltage, angle);
    //  m_rearLeft.testSet(voltage, angle);
    //  m_rearRight.testSet(voltage, angle);
    m_frontLeft.setDesiredState(new SwerveModuleState(voltage, Rotation2d.fromRadians(angle)));
    m_frontRight.setDesiredState(new SwerveModuleState(voltage, Rotation2d.fromRadians(angle)));
    m_rearRight.setDesiredState(new SwerveModuleState(voltage, Rotation2d.fromRadians(angle)));
    m_rearLeft.setDesiredState(new SwerveModuleState(voltage, Rotation2d.fromRadians(angle)));
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public void setGyroAngleDeg(double angle) {
    m_gyroOffsetDeg = angle + m_gyro.getYaw().magnitude(); // confirm magnitude gives degrees
  }

  public double getGyroAngleRadians() {
    return MathUtil.angleModulus(Math.toRadians(getGyroAngleDegrees()));
  }

  public double getGyroAngleDegrees () {
    return -m_gyro.getYaw().magnitude() + m_gyroOffsetDeg; // fix offset name
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getAngularVel()[0].magnitude(); // confirm z is the correct axis
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  public void setP(double val) {
    headingPidController.setP(val);
  }

  public void setI(double val) {
    headingPidController.setI(val);
  }

  public void setD(double val) {
    headingPidController.setD(val);
  }

  @Override
  public double getMaxSpeedLimit() {
    return DriveConstants.kMaxSpeedMetersPerSecond;
  }

  @Override
  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] { m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  };

  @Override
  public SwerveDriveKinematics getKinematics() {
    return DriveConstants.kinematics;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("gyroAngleDegrees", this::getGyroAngleDegrees, this::setGyroAngleDeg);
    builder.addDoubleProperty("MaxSpeedLimit", () -> {
      return maxSpeedLimit;
    }, (x) -> {
      maxSpeedLimit = x;
    });
    builder.addDoubleProperty("MaxRotationLimit", () -> {
      return maxRotationLimit;
    }, (x) -> {
      maxRotationLimit = x;
    });
    builder.addStringProperty("CommandedSpeeds",
                              ()->String.format("x:%5.2f y:%5.2f (%5.2f %5.2f deg) %4.2f",
                                                CommandedSpeeds.vxMetersPerSecond,
                                                CommandedSpeeds.vyMetersPerSecond,
                                                Math.hypot(CommandedSpeeds.vxMetersPerSecond, CommandedSpeeds.vyMetersPerSecond),
                                                Math.toDegrees(Math.atan2(CommandedSpeeds.vxMetersPerSecond, CommandedSpeeds.vyMetersPerSecond)),
                                                CommandedSpeeds.omegaRadiansPerSecond
                                                ),
                              null
                              );
  }

  @Override
  public void simulationPeriodic() {
    m_frontLeft.iterateSim(Robot.kDefaultPeriod);
    m_frontRight.iterateSim(Robot.kDefaultPeriod);
    m_rearLeft.iterateSim(Robot.kDefaultPeriod);
    m_rearRight.iterateSim(Robot.kDefaultPeriod);
  }
}