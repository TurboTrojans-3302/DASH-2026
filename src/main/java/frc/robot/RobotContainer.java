// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.function.Supplier;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.autoncommands.AutoShoot;
import frc.robot.autoncommands.AutoSideStartMoveAndShootNoNav;
import frc.robot.autoncommands.AutoShootFromCenter;
import frc.robot.autoncommands.AutoShootFromLeft;
import frc.robot.autoncommands.AutoShootFromRight;
import frc.robot.autoncommands.DoNothing;
import frc.robot.commands.MeasureAndSetRange;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Configs;
import frc.robot.subsystems.DXsensor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.GameData;
import frc.robot.subsystems.Harvester;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Shooter;
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

  private static final double kHopperNudgeIncrement = 4.0;
  private static final double kHopperNudgeOpenLoopSpeed = 0.2;

  private static RobotContainer instance;

  private Map<String, Supplier<Command>> autonCommands;

  // The robot's subsystems
  public DriveTrain m_robotDrive;
  public Harvester m_harvester;
  public Climbers m_climbers;
  public Shooter m_shooter;
  public Hopper m_hopper;
  public Navigation m_navigation;
  public DXsensor m_dxSensor;
  public GameData m_gameData;

  public PowerDistribution pdh;

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

    // The robot's subsystems
    m_robotDrive = new DriveTrain(Configs.driveConfigFolder, Constants.FieldConstants.StartCenterTouchingHub);
    SmartDashboard.putData("DriveSubsystem", m_robotDrive);

    m_dxSensor = new DXsensor(Constants.CanIds.DX_SENSOR_CAN_ID);
    SmartDashboard.putData("DXsensorSubsystem", m_dxSensor);

    m_navigation = new Navigation(m_robotDrive, m_dxSensor);
    SmartDashboard.putData("NavigationSubsystem", m_navigation);

    if (HOPPER_ENABLE) {
      m_hopper = new Hopper();
      SmartDashboard.putData("Hopper", m_hopper);
    }

    if (SHOOTER_ENABLE) {
      m_shooter = new Shooter(Constants.CanIds.kShooterMotorCanId, Constants.CanIds.kFeederMotorCanId, Constants.CanIds.kSecondFeederMotorCanId);
      SmartDashboard.putData("ShooterSubsystem", m_shooter);
    }

    if (HARVESTER_ENABLE) {
      m_harvester = new Harvester();
      SmartDashboard.putData("HarvesterSubsystem", m_harvester);
    }

    m_BlinkinLED = new REVBlinkinLED(Constants.BLINKIN_LED_PWM_CHANNEL);

    m_gameData = new GameData();
    SmartDashboard.putData("GameData", m_gameData);

    pdh = new PowerDistribution(53, ModuleType.kRev);
    SmartDashboard.putData(pdh);

    autonCommands = Map.of(
        "Do Nothing", () -> new DoNothing(),
        "Start Center", () -> new AutoShootFromCenter(m_robotDrive, m_shooter, m_navigation),
        "Start Left", () -> new AutoShootFromLeft(m_robotDrive, m_shooter, m_navigation), 
        "Start Right", () -> new AutoShootFromRight(m_robotDrive, m_shooter, m_navigation),
        "Auto Shoot", () -> new AutoShoot(m_robotDrive, m_shooter, m_navigation)
      ,
        "Fwd 1m, left 15deg", ()-> new AutoSideStartMoveAndShootNoNav(m_robotDrive, m_navigation, m_shooter, 1.0, -15.0),
        "Fwd 1m, right 15deg", ()-> new AutoSideStartMoveAndShootNoNav(m_robotDrive, m_navigation, m_shooter, 1.0, 15.0)        
      );
  }

  public void setDefaultCommands() {
    // Configure default commands
    Command teleopCommand = new TeleopDrive(m_robotDrive, m_driverController, m_navigation);
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

    JoystickButton toggleCameraStream = new JoystickButton(m_driverController, XboxController.Button.kY.value);
    toggleCameraStream.onTrue(new InstantCommand(() -> m_navigation.toggleCameraStream(), m_navigation));

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

    JoystickButton coPilotToggleCameraStream = new JoystickButton(m_copilotController, XboxController.Button.kY.value);
    coPilotToggleCameraStream.onTrue(new InstantCommand(() -> m_navigation.toggleCameraStream(), m_navigation)); 

  /**
     * Button Board
     *
     */

  JoystickButton enableDangerMode = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.SafetySwitch);

  if(SHOOTER_ENABLE)
  {
    JoystickButton increaseShooterSpeed = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Switch1Up);
    JoystickButton decreaseShooterSpeed = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Switch1Down);
    JoystickButton stopShooter = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.LeftKnobPush);
    JoystickButton enableShooterPID = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Switch4Up);
    JoystickButton disableShooterPID = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Switch4Down);

    increaseShooterSpeed.whileTrue(m_shooter.incrementSpeedCommand());
    decreaseShooterSpeed.whileTrue(m_shooter.decrementSpeedCommand());
    stopShooter.onTrue(new InstantCommand(() -> m_shooter.stop(), m_shooter));
    enableShooterPID.onTrue(new InstantCommand(() -> m_shooter.enablePID(true), m_shooter));
    disableShooterPID.onTrue(new InstantCommand(() -> m_shooter.enablePID(false), m_shooter));

    // toggle between using timer to limit feeder and ignoring timer (feeder is
    // always active)
    Trigger scoringAllowed = new Trigger(() -> m_gameData.scoring());
    JoystickButton shootButton = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.EngineStart); // into shooter
    JoystickButton feederReverse = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Right1); // feed reverse to
                                                                                                    // dislodge
    JoystickButton feederForward = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Right3); // blockage
    JoystickButton spinUpShooter = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Left1); // spin up shooter
                                                                                                   // without feeding
    enableDangerMode.onTrue(new InstantCommand(() -> m_shooter.setDangerMode(!m_shooter.isDangerMode()), m_shooter));

    shootButton.and(scoringAllowed.or(() -> m_shooter.isDangerMode())).whileTrue(m_shooter.shootCommand());

    feederReverse.whileTrue(m_shooter.reverseFeedCommand());
    feederForward.whileTrue(m_shooter.forwardFeedCommand());
    spinUpShooter.onTrue(new MeasureAndSetRange(m_navigation, m_shooter).withTimeout(2.0));

  }

  if(HOPPER_ENABLE)
  {
    // Hopper controls:
    JoystickButton hopperExpand = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Left2);
    JoystickButton hopperRetract = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Right2);
    hopperExpand.onTrue(m_hopper.expandCommand());
    if (HARVESTER_ENABLE) {
      hopperRetract.and(enableDangerMode.or(() -> !m_harvester.isOn())).onTrue(m_hopper.retractCommand());
    } else {
      hopperRetract.onTrue(m_hopper.retractCommand());
    }

    Trigger nudgeHopperLeftOut = new Trigger(() -> m_buttonBoard.getPOV() == OIConstants.ButtonBox.StickUpLeft);
    Trigger nudgeHopperOut = new Trigger(() -> m_buttonBoard.getPOV() == OIConstants.ButtonBox.StickUp);
    Trigger nudgeHopperRightOut = new Trigger(() -> m_buttonBoard.getPOV() == OIConstants.ButtonBox.StickUpRight);
    Trigger nudgeHopperLeftIn = new Trigger(() -> m_buttonBoard.getPOV() == OIConstants.ButtonBox.StickDownLeft);
    Trigger nudgeHopperIn = new Trigger(() -> m_buttonBoard.getPOV() == OIConstants.ButtonBox.StickDown);
    Trigger nudgeHopperRightIn = new Trigger(() -> m_buttonBoard.getPOV() == OIConstants.ButtonBox.StickDownRight);
    nudgeHopperOut.and(() -> m_hopper.isPIDEnabled()).whileTrue(m_hopper.nudgeCommand(1));
    nudgeHopperIn.and(() -> m_hopper.isPIDEnabled()).whileTrue(m_hopper.nudgeCommand(-1));

    nudgeHopperLeftOut.and(() -> !m_hopper.isPIDEnabled())
        .whileTrue(m_hopper.manualMoveCommand(() -> kHopperNudgeOpenLoopSpeed, () -> 0.0, enableDangerMode));
    nudgeHopperOut.and(() -> !m_hopper.isPIDEnabled())
        .whileTrue(m_hopper.manualMoveCommand(() -> kHopperNudgeOpenLoopSpeed, () -> kHopperNudgeOpenLoopSpeed, enableDangerMode));
    nudgeHopperRightOut.and(() -> !m_hopper.isPIDEnabled())
        .whileTrue(m_hopper.manualMoveCommand(() -> 0.0, () -> kHopperNudgeOpenLoopSpeed, enableDangerMode));
    nudgeHopperLeftIn.and(() -> !m_hopper.isPIDEnabled())
        .whileTrue(m_hopper.manualMoveCommand(() -> -kHopperNudgeOpenLoopSpeed, () -> 0.0, enableDangerMode));
    nudgeHopperIn.and(() -> !m_hopper.isPIDEnabled())
        .whileTrue(m_hopper.manualMoveCommand(() -> -kHopperNudgeOpenLoopSpeed, () -> -kHopperNudgeOpenLoopSpeed, enableDangerMode));
    nudgeHopperRightIn.and(() -> !m_hopper.isPIDEnabled())
        .whileTrue(m_hopper.manualMoveCommand(() -> 0.0, () -> -kHopperNudgeOpenLoopSpeed, enableDangerMode));

    JoystickButton hopperPIDenable = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Switch3Up);
    JoystickButton hopperPIDdisable = new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Switch3Down);
    hopperPIDenable.onTrue(new InstantCommand(() -> m_hopper.setPIDEnabled(true), m_hopper));
    hopperPIDdisable.onTrue(new InstantCommand(() -> m_hopper.setPIDEnabled(false), m_hopper));
  }

  if(HARVESTER_ENABLE)
  {
    new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.RightKnobCW)
        .whileTrue(new RunCommand(() -> m_harvester.adjustPullInRPM(1), m_harvester));
    new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.RightKnobCCW)
        .whileTrue(new RunCommand(() -> m_harvester.adjustPullInRPM(-1), m_harvester));

    new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Switch2Up)
        .onTrue(new InstantCommand(() -> m_harvester.enablePID(true), m_harvester));
    new JoystickButton(m_buttonBoard, OIConstants.ButtonBox.Switch2Down)
        .onTrue(new InstantCommand(() -> m_harvester.enablePID(false), m_harvester));
  }
  }

  public void configureTestControls() {
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
 
  public void onDSAttached(){
    readPIDswitches();
  }

  public void readPIDswitches() {
    // Read the actual switch state at binding time so PID starts in the correct mode
    if (HOPPER_ENABLE){
      if(m_buttonBoard.getRawButton(OIConstants.ButtonBox.Switch3Up)){
        m_hopper.setPIDEnabled(true);
      }
      if(m_buttonBoard.getRawButton(OIConstants.ButtonBox.Switch3Down)){
        m_hopper.setPIDEnabled(false);
      }
    }
    if(SHOOTER_ENABLE){
      if(m_buttonBoard.getRawButton(OIConstants.ButtonBox.Switch4Up)){
        m_shooter.enablePID(true);
      }
      if(m_buttonBoard.getRawButton(OIConstants.ButtonBox.Switch4Down)){
        m_shooter.enablePID(false);
      }
    }

    if(HARVESTER_ENABLE){
      if(m_buttonBoard.getRawButton(OIConstants.ButtonBox.Switch2Up)){
        m_harvester.enablePID(true);
      }
      if(m_buttonBoard.getRawButton(OIConstants.ButtonBox.Switch2Down)){
        m_harvester.enablePID(false);
      }
    }
  }

  public void saveSomePreferences() {
    if (HOPPER_ENABLE) m_hopper.savePositions();
  }

  public SendableChooser<String> createAutonomousChooser() {
    final String prefKey = "selectedAutonRoutine";
    SendableChooser<String> chooser = new SendableChooser<>();
    
    for (Map.Entry<String, Supplier<Command>> entry : autonCommands.entrySet()) {
      chooser.addOption(entry.getKey(), entry.getKey());
    }

    if(Preferences.containsKey(prefKey)) {
      String routineName = Preferences.getString(prefKey, null);
      if(autonCommands.containsKey(routineName)) {
        chooser.setDefaultOption(routineName, routineName);
      }else{
        System.out.println("Warning: saved auton routine '" + routineName + "' not found."); 
      }
    }

    chooser.onChange((selectedName)->{
      Preferences.setString(prefKey, selectedName);
    });

    return chooser;
  }

  public Command getAutonomousCommand(String selectedRoutineName) {
    if (selectedRoutineName != null && autonCommands.containsKey(selectedRoutineName)) {
      return autonCommands.get(selectedRoutineName).get();
    } else {
      System.out.println(
          "Warning: selected autonomous routine '" + selectedRoutineName + "' not found. Defaulting to Do Nothing.");
      return new DoNothing();
    }
  }

}