// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Navigation;

public class TeleopDrive extends Command {
  private DriveTrain m_robotDrive;
  private XboxController m_driverController;
  private Navigation m_nav;
  private boolean m_fieldOrientedEnable = true;
  private boolean m_slowDriveFlag = false;
  private boolean m_DpadDriveFlag = false;
  private final double kDPADdriveSpeed = 2.0; // m/s, speed when driving strictly north/south/east/west with field-oriented control, can be tuned based on driver preference

  /** Creates a new TeleopDrive. */
  public TeleopDrive(DriveTrain robotDrive, XboxController driverController, Navigation nav) {
    m_driverController = driverController;
    m_robotDrive = robotDrive;
    m_nav = nav;
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_robotDrive.lock();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_driverController.getRightStickButtonPressed()) {
      m_fieldOrientedEnable = !m_fieldOrientedEnable;
    }

    if (m_driverController.getLeftStickButtonPressed()) {
      m_slowDriveFlag = !m_slowDriveFlag;
    }
    double speedScale = m_slowDriveFlag ? 0.5 : 1.0;

    double forward = m_robotDrive.getMaxSpeed() * stick2speed(speedScale * m_driverController.getLeftY());
    double leftward = m_robotDrive.getMaxSpeed() * stick2speed(speedScale * m_driverController.getLeftX());
    double rotate = m_robotDrive.getMaxAngularVelocity() * stick2speed(speedScale * m_driverController.getRightX());


    if (m_fieldOrientedEnable) {
      if(m_driverController.getPOV() != -1 && m_DpadDriveFlag){
        double povAngle = Math.toRadians(m_driverController.getPOV());
        forward = kDPADdriveSpeed * Math.cos(povAngle);
        leftward = kDPADdriveSpeed * -Math.sin(povAngle);
      }

      if(m_driverController.getAButton()){
        Rotation2d desiredAngle = m_nav.getAbsBearingToTarget().plus(Rotation2d.k180deg);
        System.out.println("Desired Angle: " + desiredAngle.getDegrees());
        m_robotDrive.driveHeading(new Translation2d(forward, leftward), desiredAngle);
      } else {
        m_robotDrive.drive(new Translation2d(forward, leftward), rotate, true);
      }
    } else {
      m_robotDrive.driveRobotOriented(forward, leftward, rotate);
    }

  }

  // applies deadband and scaling to raw stick value
  private double stick2speed(double stickValue) {
    return -Math.signum(stickValue) * Math.pow(MathUtil.applyDeadband(stickValue, OIConstants.kDriveDeadband), 2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty("FieldOrientedEnable", () -> m_fieldOrientedEnable, (x) -> {
      m_fieldOrientedEnable = x;
    });
    builder.addBooleanProperty("SlowDriveFlag", () -> m_slowDriveFlag, (x) -> {
      m_slowDriveFlag = x;
    });
    builder.addBooleanProperty("DpadDriveFlag", () -> m_DpadDriveFlag, (x) -> {
      m_DpadDriveFlag = x;
    });
  }
}