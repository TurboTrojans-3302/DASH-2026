// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Shoot;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Configs;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hopper;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static boolean INTAKE_ENABLE = true;
  private static boolean SHOOTER_ENABLE = true;
  public static boolean feederEnabled = true;
  public static boolean ignorePeriods = false;

  private static RobotContainer instance;

  // The robot's subsystems
  public DriveTrain m_robotDrive;
  public Intake m_intake;
  public IntakeArm m_intakeArm;
  public Climbers m_climbers;
  public Shooter m_shooter;
  public Hopper m_hopper;

  private SendableChooser<Command> m_autonomousChooser = new SendableChooser<Command>();
  private SendableChooser<Pose2d> m_startPosChooser = new SendableChooser<Pose2d>();
  private Command m_autonCommand;

  private final REVBlinkinLED m_BlinkinLED;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
//  XboxController m_copilotController = new XboxController(OIConstants.kCopilotControllerPort);
  GenericHID m_buttonBoard = new GenericHID(OIConstants.kButtonBoardPort);


  public int targetTagId = 0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    instance = this;

    // CameraServer.startAutomaticCapture();

    // The robot's subsystems
    m_robotDrive = new DriveTrain(Configs.driveConfigFolder);
    SmartDashboard.putData("DriveSubsystem", m_robotDrive);

    

    m_hopper = new Hopper();
    SmartDashboard.putData("Hopper", m_hopper);

    SmartDashboard.putString("TeleOp Shift", Robot.getInstance().getCurrentShiftName());
    SmartDashboard.putNumber("Time Left In Shift:", Robot.getInstance().getTimeLeftInShift());
    SmartDashboard.putBoolean("Score", Robot.getInstance().scoring()); // tower activated, robot can score

    if(SHOOTER_ENABLE){
      m_shooter = new Shooter(Constants.CanIds.kShooterMotorCanId, Constants.CanIds.kFeederMotorCanId);
      SmartDashboard.putData("ShooterSubsystem", m_shooter);
    }

    m_BlinkinLED = new REVBlinkinLED(Constants.BLINKIN_LED_PWM_CHANNEL);
  }

  public void setDefaultCommands() {
    // Configure default commands
    Command teleopCommand = new TeleopDrive(m_robotDrive, m_driverController);
    m_robotDrive.setDefaultCommand(teleopCommand);
    SmartDashboard.putData("TeleopCommand", teleopCommand);

    m_shooter.setDefaultCommand(new InstantCommand(() -> {
      m_shooter.setFeederSpeed(0.0);
    }));
    // Keep hopper motors idle when no commands are active
    m_hopper.setDefaultCommand(new RunCommand(() -> m_hopper.stop(), m_hopper));
  }

  public static RobotContainer getInstance() {
    return instance;
  }

  public void configureButtonBindings() {

    /**
     * Driver's Controller
     */

    if (INTAKE_ENABLE) {

    }

    /**
     * Copilot's Controller
     *
     */

    if (SHOOTER_ENABLE) {
      JoystickButton increaseShooterSpeed = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.LeftKnobCCW);
      JoystickButton decreaseShooterSpeed = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.LeftKnobCW);
      JoystickButton stopShooter = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.LeftKnobPush);
      JoystickButton enablePID = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Switch4Up);
      JoystickButton disablePID = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Switch4Down);

      increaseShooterSpeed.whileTrue(m_shooter.incrementSpeedCommand());
      decreaseShooterSpeed.whileTrue(m_shooter.decrementSpeedCommand());
      stopShooter.onTrue(new InstantCommand(() -> m_shooter.stop(), m_shooter));
      enablePID.onTrue(new InstantCommand(() -> m_shooter.enablePID(true), m_shooter));
      disablePID.onTrue(new InstantCommand(() -> m_shooter.enablePID(false), m_shooter ));


      // toggle between using timer to limit feeder and ignoring timer (feeder is
      // always active)
      JoystickButton toggleTimerUsage = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.SafetySwitch);
      Trigger scoringAllowed = new Trigger(() -> Robot.getInstance().scoring());
      JoystickButton feedShooter = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.EngineStart); // into shooter
      JoystickButton feederReverse = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Right1); // feed reverse to
                                                                                                      // disloge
                                                                                                      // blockage
      JoystickButton spinUpShooter = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Left1); // spin up shooter
                                                                                                     // without feeding

      feedShooter.and(scoringAllowed.or(toggleTimerUsage)).whileTrue(new Shoot(m_shooter));

      feederReverse.whileTrue(
          new RunCommand(() -> m_shooter.setFeederSpeed(-Constants.ShooterConstants.feederSpeedDefault), m_shooter));
      spinUpShooter
          .onTrue(new InstantCommand(() -> m_shooter.setRPMsetpoint(Constants.ShooterConstants.defaultShootRPM)));

    }

    // Hopper controls: Left2 expand, Right2 retract while held
    JoystickButton hopperExpand = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Left2);
    JoystickButton hopperRetract = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Right2);

    hopperExpand.whileTrue(m_hopper.expandCommand());
    hopperRetract.whileTrue(m_hopper.retractCommand());

  }

  public void configureTestControls() {
    JoystickButton testPlus = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Switch1Up);
    JoystickButton testMinus = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Switch1Down);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    System.out.println("getAutonomousCommand()");
    return m_autonCommand;
  }

  public void setAutonCommand(Command cmd) {
    m_autonCommand = cmd;
    System.out.println("setAutonCommand" + m_autonCommand);
  }

  public Pose2d getStartPosition() {
    return m_startPosChooser.getSelected();
  }

  public void setLED(double value) {
    m_BlinkinLED.set(value);
  }

  /*
   * called once when is set to Red by the DriverStation
   */
  public void initRed() {
    m_robotDrive.zeroGyroWithAlliance();
    /*
     * m_autonomousChooser = AutonMenus.getRed();
     * SmartDashboard.putData("Auton Command", m_autonomousChooser);
     * m_autonomousChooser.onChange(this::setAutonCommand);
     * 
     * m_startPosChooser = StartPositions.getRed();
     * SmartDashboard.putData("Start Position", m_startPosChooser);
     * m_startPosChooser.onChange(this::setStartPosition);
     */
  }

  /*
   * called once when is set to Blue by the DriverStation
   */
  public void initBlue() {
    m_robotDrive.zeroGyroWithAlliance();
    /*
     * m_autonomousChooser = AutonMenus.getBlue();
     * SmartDashboard.putData("Auton Command", m_autonomousChooser);
     * m_autonomousChooser.onChange((this::setAutonCommand));
     * 
     * m_startPosChooser = StartPositions.getBlue();
     * SmartDashboard.putData("Start Position", m_startPosChooser);
     * m_startPosChooser.onChange(this::setStartPosition);
     */
  }

  // private void setStartPosition(Pose2d pose) {
  // if (DriverStation.isDisabled()) {
  // System.out.println("setStartPosition()" + pose.toString());
  // m_robotDrive.setGyroAngleDeg(pose.getRotation().getDegrees());
  // m_nav.resetOdometry(pose);
  // }
  // }

}