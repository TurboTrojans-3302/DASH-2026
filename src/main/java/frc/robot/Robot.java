// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.Map;
import java.util.Optional;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static Robot instance;
  public static DriverStation.Alliance alliance;

  private RobotContainer m_robotContainer;
  private boolean gameDataReceived = false;

  private SendableChooser<String> m_autonomousChooser;
  private Command m_autonomousCommand;

  Robot(){
    instance = this;
  }

  Robot(double period) {
    super(period);
    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Starts recording to data log
    DataLogManager.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_robotContainer.configureButtonBindings();
    m_robotContainer.setDefaultCommands();
    //DataLogManager.start();
    //CanBridge.runTCP();
    LimelightHelpers.setCameraPose_RobotSpace(Constants.LimelightConstants.name,
                                              Constants.LimelightConstants.Offset.forward,
                                              Constants.LimelightConstants.Offset.side,
                                              Constants.LimelightConstants.Offset.up,
                                              Constants.LimelightConstants.Offset.roll,
                                              Constants.LimelightConstants.Offset.pitch,
                                              Constants.LimelightConstants.Offset.yaw
                                            );

    
    m_autonomousChooser = m_robotContainer.createAutonomousChooser();
    SmartDashboard.putData("Autonomous", m_autonomousChooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * Dashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.saveSomePreferences();
    m_robotContainer.setLED(REVBlinkinLED.Pattern.SOLID_VIOLET);
    m_autonomousChooser = m_robotContainer.createAutonomousChooser();
    SmartDashboard.putData("Autonomous", m_autonomousChooser);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void driverStationConnected() {
    m_robotContainer.onDSAttached();
  }

  @Override
  public void disabledPeriodic() {
    if(alliance == null) {
      Optional<Alliance> a = DriverStation.getAlliance();
      if (a.isPresent()) {
        alliance = a.get();
        if(alliance == Alliance.Red) {
          m_robotContainer.initRed();
        } else {
          m_robotContainer.initBlue();
        }
      }
    }
  }


  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    setLED(LEDmode.Auton);

    String commandName = m_autonomousChooser.getSelected();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(commandName);
    System.out.println("Starting command: " + commandName + " -> " + m_autonomousCommand.getName());
    CommandScheduler.getInstance().schedule(m_autonomousCommand);

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    
    setLED(LEDmode.Teleop);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    m_robotContainer.m_shooter.stop();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    
    String gamedatastring = DriverStation.getGameSpecificMessage();

    if (gamedatastring.length() > 0 && !gameDataReceived) {
      RobotContainer.getInstance().m_gameData.setGameDataSring(gamedatastring);
      gameDataReceived = true;
    }
  } 

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    m_robotContainer.m_robotDrive.lock();

    m_robotContainer.configureTestControls();
                           
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
    m_robotContainer.m_robotDrive.removeDefaultCommand();
  }

  private enum LEDmode {
    Auton,
    Teleop,
    HaveNote,
    Ready2Shoot
  }

  Map<Alliance, Map<LEDmode, Double>> ledPatternMap = Map.of(
      Alliance.Red, Map.of(
          LEDmode.Auton, REVBlinkinLED.Pattern.COLOR1_LARSON_SCANNER,
          LEDmode.Teleop, REVBlinkinLED.Pattern.SOLID_RED,
          LEDmode.HaveNote, REVBlinkinLED.Pattern.COLOR1_HEARTBEAT_MEDIUM,
          LEDmode.Ready2Shoot, REVBlinkinLED.Pattern.COLOR1_HEARTBEAT_FAST),
      Alliance.Blue, Map.of(
          LEDmode.Auton, REVBlinkinLED.Pattern.COLOR2_LARSON_SCANNER,
          LEDmode.Teleop, REVBlinkinLED.Pattern.SOLID_BLUE,
          LEDmode.HaveNote, REVBlinkinLED.Pattern.COLOR2_HEARTBEAT_MEDIUM,
          LEDmode.Ready2Shoot, REVBlinkinLED.Pattern.COLOR2_HEARTBEAT_FAST));

  private void setLED(LEDmode mode) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      m_robotContainer.setLED(ledPatternMap.get(alliance.get()).get(mode));
    } else {
      m_robotContainer.setLED(REVBlinkinLED.Pattern.COLOR1_AND_2_TWINKLES_COLOR1_AND_2);
    }
  }
}