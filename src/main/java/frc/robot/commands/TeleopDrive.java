// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.DriveConstants;


public class TeleopDrive extends Command {
  private DriveSubsystem m_robotDrive;
  private XboxController m_driverController;
  private boolean m_fieldOrientedEnable = true;
  private boolean m_slowDriveFlag = false;
  private double orbitSpeed = DriveConstants.ORBIT_SPEED;
  DriveSubsystem drive;
  Navigation nav;

  /** Creates a new TeleopDrive. */
  public TeleopDrive(DriveSubsystem robotDrive, XboxController driverController) {
    m_driverController = driverController;
    m_robotDrive = robotDrive;
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_robotDrive.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(m_driverController.getRightStickButtonPressed()){
      m_fieldOrientedEnable = !m_fieldOrientedEnable;
    }
    
    if (m_driverController.getLeftStickButtonPressed()) {
      m_slowDriveFlag = !m_slowDriveFlag;
    }
    double speedScale = m_slowDriveFlag ? 0.5 : 1.0;

    double forward = stick2speed(speedScale * m_driverController.getLeftY());
    double leftward = stick2speed(speedScale * m_driverController.getLeftX());
    double rotate = stick2speed(speedScale * m_driverController.getRightX());

    if(forward == 0.0 && leftward == 0.0 && rotate == 0.0){

      if(m_driverController.getXButton()){
        //new OrbitAroundReef(drive, nav, -orbitSpeed * speedScale);
      }
      else if(m_driverController.getBButton()){
      //  new OrbitAroundReef(drive, nav, orbitSpeed * speedScale);
      }
      else{
        m_robotDrive.stop();
      }

      final Translation2d orbitCenter = new Translation2d(1.0, 0.0);
      if(m_driverController.getPOV() == 90 && m_driverController.getXButton() == false && m_driverController.getBButton() == false){
        m_robotDrive.orbitRobotFrame(orbitSpeed * -speedScale, orbitCenter);
      }
      else if(m_driverController.getPOV() == 270){
        m_robotDrive.orbitRobotFrame(orbitSpeed * speedScale, orbitCenter);
      }else{
        m_robotDrive.stop();
      }

    } else {

      if(m_fieldOrientedEnable) {
        double reverse = (Robot.alliance == Alliance.Red) ? -1.0 : 1.0;
        m_robotDrive.driveFieldOriented(reverse * forward, reverse * leftward, rotate);
      }else{
        m_robotDrive.driveRobotOriented(forward, leftward, rotate);
      }

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
    builder.addBooleanProperty("FieldOrientedEnable", () -> m_fieldOrientedEnable, (x)->{m_fieldOrientedEnable = x;});
    builder.addBooleanProperty("SlowDriveFlag", () -> m_slowDriveFlag, (x)->{m_slowDriveFlag = x;});
    builder.addDoubleProperty("Orbit Speed", ()-> orbitSpeed, (x)-> orbitSpeed = x);
  }
}