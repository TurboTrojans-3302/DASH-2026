// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Configs;
import frc.robot.subsystems.DXsensor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Harvester;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Shooter;
import frc.robot.autoncommands.DoNothing;
import frc.robot.subsystems.GameData;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static boolean HARVESTER_ENABLE = true;
  private static boolean CLIMBERS_ENABLE = false;
  private static boolean HOPPER_ENABLE = true;
  private static boolean SHOOTER_ENABLE = true;

  public static boolean ignorePeriods = false;

  private static RobotContainer instance;

  // The robot's subsystems
  public DriveTrain m_robotDrive;
  public Harvester m_harvester;
  public Climbers m_climbers;
  public Shooter m_shooter;
  public Hopper m_hopper;
  public Navigation m_navigation;
  public DXsensor m_dxSensor;
  public GameData m_gameData;

  private SendableChooser<Command> m_autonomousChooser = new SendableChooser<Command>();
  private SendableChooser<Pose2d> m_startPosChooser = new SendableChooser<Pose2d>();
  //private Command m_autonCommand;

  private final REVBlinkinLED m_BlinkinLED;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_copilotController = new XboxController(OIConstants.kCopilotControllerPort);
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

    m_navigation = new Navigation(m_robotDrive);
    SmartDashboard.putData("NavigationSubsystem", m_navigation);

    if(HOPPER_ENABLE){
      m_hopper = new Hopper();
      SmartDashboard.putData("Hopper", m_hopper);
    }

    if(SHOOTER_ENABLE){
      m_shooter = new Shooter(Constants.CanIds.kShooterMotorCanId, Constants.CanIds.kFeederMotorCanId);
      SmartDashboard.putData("ShooterSubsystem", m_shooter);
    }

    if(HARVESTER_ENABLE){
      m_harvester = new Harvester(m_robotDrive, m_hopper);
      SmartDashboard.putData("HarvesterSubsystem", m_harvester);
    }

    m_dxSensor = new DXsensor(Constants.CanIds.DX_SENSOR_CAN_ID);
    SmartDashboard.putData("DXsensorSubsystem", m_dxSensor);

    m_BlinkinLED = new REVBlinkinLED(Constants.BLINKIN_LED_PWM_CHANNEL);

    m_autonomousChooser.setDefaultOption("Do Nothing", new DoNothing());
    SmartDashboard.putData("Autonomous Chooser", m_autonomousChooser);

    m_gameData = new GameData();
    SmartDashboard.putData("GameData", m_gameData);
  }

  public void setDefaultCommands() {
    // Configure default commands
    Command teleopCommand = new TeleopDrive(m_robotDrive, m_driverController);
    m_robotDrive.setDefaultCommand(teleopCommand);
    SmartDashboard.putData("TeleopCommand", teleopCommand);
  }
  
  public static RobotContainer getInstance() {
    return instance;
  }

  public void configureButtonBindings() {

    /**
     * Driver's Controller
     */

    if (HARVESTER_ENABLE) {
      Trigger harvestForward = new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.1);
      Trigger harvestReverse = new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.1);
      harvestForward.whileTrue(m_harvester.PullInCommand());
      harvestReverse.whileTrue(m_harvester.PushOutCommand());
    }

    if (HOPPER_ENABLE){
      Trigger extendHopper = new Trigger(() -> m_driverController.getRightBumper());
      Trigger retractHopper = new Trigger(() -> m_driverController.getLeftBumper());
      extendHopper.onTrue(m_hopper.expandCommand());
      retractHopper.onTrue(m_harvester.StopCommand().andThen(m_hopper.retractCommand()));      
    }
    /**
     * Copilot's Controller
     *
     */

     if (HARVESTER_ENABLE) {
      Trigger copilotHarvest = new Trigger(() -> m_copilotController.getLeftTriggerAxis() > 0.1);
      Trigger copilotHarvestReverse = new Trigger(() -> m_copilotController.getRightTriggerAxis() > 0.1);
      copilotHarvest.whileTrue(m_harvester.PullInCommand());
      copilotHarvestReverse.whileTrue(m_harvester.PushOutCommand());
    }

    if (HOPPER_ENABLE){
      Trigger extendHopperCopilot = new Trigger(() -> m_copilotController.getRightBumper());
      Trigger retractHopperCopilot = new Trigger(() -> m_copilotController.getLeftBumper());
      extendHopperCopilot.onTrue(m_hopper.expandCommand());
      retractHopperCopilot.onTrue(m_harvester.StopCommand().andThen(m_hopper.retractCommand()));
      final double hopperManualJoystickDeadband = 0.1;
      Trigger hopperManualControl = new Trigger(() -> Math.abs(m_copilotController.getLeftY()) > hopperManualJoystickDeadband);
      hopperManualControl.whileTrue(m_hopper.manualMoveCommand(() -> MathUtil.applyDeadband(-m_copilotController.getLeftY() * .5, hopperManualJoystickDeadband)));
    }
    

    /**
     * Button Board
     *
     */

    if (SHOOTER_ENABLE) {
      JoystickButton increaseShooterSpeed = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.LeftKnobCCW);
      JoystickButton decreaseShooterSpeed = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.LeftKnobCW);
      JoystickButton stopShooter = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.LeftKnobPush);
      JoystickButton enableShooterPID = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Switch4Up);
      JoystickButton disableShooterPID = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Switch4Down);

      increaseShooterSpeed.whileTrue(m_shooter.incrementSpeedCommand());
      decreaseShooterSpeed.whileTrue(m_shooter.decrementSpeedCommand());
      stopShooter.onTrue(new InstantCommand(() -> m_shooter.stop(), m_shooter));
      enableShooterPID.onTrue(new InstantCommand(() -> m_shooter.enablePID(true), m_shooter));
      disableShooterPID.onTrue(new InstantCommand(() -> m_shooter.enablePID(false), m_shooter ));

      
      // toggle between using timer to limit feeder and ignoring timer (feeder is
      // always active)
      JoystickButton enableDangerMode = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.SafetySwitch);
      Trigger scoringAllowed = new Trigger(() -> m_gameData.scoring());
      JoystickButton feedShooter = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.EngineStart); // into shooter
      JoystickButton feederReverse = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Right1); // feed reverse to dislodge                                                                                                                                                                            // blockage
      JoystickButton spinUpShooter = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Left1); // spin up shooter
                                                                                                     // without feeding
      enableDangerMode.onTrue(new InstantCommand(() -> m_shooter.setDangerMode(!m_shooter.isDangerMode()), m_shooter));

      feedShooter.and(scoringAllowed.or(() -> m_shooter.isDangerMode())).whileTrue(m_shooter.shootCommand());

      feederReverse.whileTrue(m_shooter.reverseFeedCommand());
      spinUpShooter.onTrue(m_shooter.spinUpCommand(() -> Constants.ShooterConstants.defaultShootRPM));

    }

    if(HOPPER_ENABLE){
      // Hopper controls: 
      JoystickButton hopperExpand = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Left2);
      JoystickButton hopperRetract = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Right2);
      hopperExpand.onTrue(m_hopper.expandCommand());
      hopperRetract.onTrue(m_hopper.retractCommand());

      JoystickButton nudgeHopperOut = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.RightKnobCW);
      JoystickButton nudgeHopperIn = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.RightKnobCCW);
      nudgeHopperOut.whileTrue(m_hopper.nudgeCommand(4));
      nudgeHopperIn.whileTrue(m_hopper.nudgeCommand(-4));

      JoystickButton hopperPIDenable = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Switch3Up);
      JoystickButton hopperPIDdisable = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Switch3Down);
      hopperPIDenable.onTrue(new InstantCommand(() -> m_hopper.setPIDEnabled(true), m_hopper));
      hopperPIDdisable.onTrue(new InstantCommand(() -> m_hopper.setPIDEnabled(false), m_hopper));

      
    }
  }

  public void configureTestControls() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    System.out.println("getAutonomousCommand()");
    return m_autonomousChooser.getSelected();
  }

  // public void setAutonCommand(Command cmd) {
  //   m_autonCommand = cmd;
  //   System.out.println("setAutonCommand" + m_autonCommand);
  // }

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
    m_navigation.setAlliance(Alliance.Red);
  }

  /*
   * called once when is set to Blue by the DriverStation
   */
  public void initBlue() {
    m_navigation.setAlliance(Alliance.Blue);
  }

  public void onDSAttached() {
    // Read the actual switch state at binding time so PID starts in the correct mode
      if(m_buttonBoard.getRawButton(OIConstants.ButtonBox.Switch3Up)){
        m_hopper.setPIDEnabled(true);
      }
      if(m_buttonBoard.getRawButton(OIConstants.ButtonBox.Switch3Down)){
        m_hopper.setPIDEnabled(false);
      }

      if(SHOOTER_ENABLE){
        if(m_buttonBoard.getRawButton(OIConstants.ButtonBox.Switch4Up)){
          m_shooter.enablePID(true);
        }
        if(m_buttonBoard.getRawButton(OIConstants.ButtonBox.Switch4Down)){
          m_shooter.enablePID(false);
        }
      }
  }
}