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
import frc.robot.subsystems.DriveTrain;



public class TeleopDrive extends Command {
  private DriveTrain m_robotDrive;
  private XboxController m_driverController;
  private boolean m_fieldOrientedEnable = false; //TODO default this to true when it's working
  private boolean m_slowDriveFlag = false;


  /** Creates a new TeleopDrive. */
  public TeleopDrive(DriveTrain robotDrive, XboxController driverController) {
    m_driverController = driverController;
    m_robotDrive = robotDrive;
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

  
    if(m_fieldOrientedEnable) {
      double reverse = (Robot.alliance == Alliance.Red) ? -1.0 : 1.0;
      m_robotDrive.drive(new Translation2d(reverse * forward, reverse * leftward), rotate, true);
    }else{
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
    builder.addBooleanProperty("FieldOrientedEnable", () -> m_fieldOrientedEnable, (x)->{m_fieldOrientedEnable = x;});
    builder.addBooleanProperty("SlowDriveFlag", () -> m_slowDriveFlag, (x)->{m_slowDriveFlag = x;});
  }
}