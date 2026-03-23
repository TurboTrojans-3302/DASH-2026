// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.PWMChannels;


public class Climber extends SubsystemBase{

  public static double START = ClimberConstants.START;
  public static double HANG = ClimberConstants.HANG;
  public static double HIGH = ClimberConstants.HIGH;  

  private SparkMax climberMotor;
  private double climberSetpoint = 0;
  private ProfiledPIDController climberPID;
  private boolean PIDEnabled = true;
  private Servo servoLeft;
  private Servo servoRight;
  private Timer servoTimer;
  private DigitalInput lowerLimitSwitch;
  private double leftHookRetractedAngle = ClimberConstants.leftHookRetractedAngle;
  private double leftHookDeployedAngle = ClimberConstants.leftHookDeployedAngle;
  private double rightHookRetractedAngle = ClimberConstants.rightHookRetractedAngle;
  private double rightHookDeployedAngle = ClimberConstants.rightHookDeployedAngle;
  private double servoTime = ClimberConstants.climberServoTime;
  private double climberLowerSoftLimit = ClimberConstants.climberLowerSoftLimit;
  private double climberUpperSoftLimit = ClimberConstants.climberUpperSoftLimit;
  private boolean limitsEnabled = true;
  private double nudgeIncrement = ClimberConstants.climberNudgeIncrement;

  private RelativeEncoder climberEncoder;




  public Climber() {
    climberMotor = new SparkMax(Constants.CanIds.kClimbMotor, MotorType.kBrushless);
    climberMotor.configure(new SparkMaxConfig().inverted(false)
        .idleMode(IdleMode.kBrake),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
    climberPID = new ProfiledPIDController(
        ClimberConstants.kPdefault,
        ClimberConstants.kIdefault,
        ClimberConstants.kDdefault,
        new TrapezoidProfile.Constraints(
            ClimberConstants.climberMaxVelocityDefault,
            ClimberConstants.climberMaxAccelerationDefault));
    climberPID.setTolerance(ClimberConstants.kToleranceDefault);
    climberPID.reset(getClimberPosition());
    climberEncoder = climberMotor.getEncoder();
    climberEncoder.setPosition(0.0);

    servoLeft = new Servo(PWMChannels.PWMServoLeft);
    servoRight = new Servo(PWMChannels.PWMServoRight);

    lowerLimitSwitch = new DigitalInput(Constants.DigitalIO.kClimberLimitSwitch);
    servoTimer = new Timer();

    loadPreferences();

    retractHooks();
  }

  public boolean atLowerSoftLimit(){
    return getClimberPosition() <= climberLowerSoftLimit && limitsEnabled;
  }

  public boolean atUpperSoftLimit(){
    return getClimberPosition() >= climberUpperSoftLimit && limitsEnabled;
  }

  public boolean waitForServo(){
    return servoTimer.hasElapsed(0.5);
  }
  public boolean atLowerHardLimit(){
    return !lowerLimitSwitch.get();
  }

  public void retractHooks(){
    servoLeft.setAngle(leftHookRetractedAngle);
    servoRight.setAngle(rightHookRetractedAngle);
    servoTimer.restart();
  }

  private void deployHooks(){
    servoLeft.setAngle(leftHookDeployedAngle);
    servoRight.setAngle(rightHookDeployedAngle);
    servoTimer.restart();
  }

  private boolean hooksReady(){
    return servoTimer.hasElapsed(servoTime);
  }

  public String hookStatus(){
    if(!hooksReady()){
      return "moving";
    } else if ((servoLeft.getAngle() == leftHookRetractedAngle) && (servoRight.getAngle() == rightHookRetractedAngle)){
      return "retracted";
    } else if ((servoLeft.getAngle() == leftHookDeployedAngle) && (servoRight.getAngle() == rightHookDeployedAngle)){
      return "deployed";
    } else {
      return "-----";
    }
  }


  public void moveClimberPID(double setpoint){
  climberSetpoint = MathUtil.clamp(setpoint, climberLowerSoftLimit, climberUpperSoftLimit);
  climberPID.setGoal(climberSetpoint);
  }

  public boolean atSetpoint(){
  return climberPID.atGoal();
  }


  public void stopClimbers(){
    climberMotor.stopMotor();
  }

  public void hold(){
    moveClimberPID(climberEncoder.getPosition());
    climberPID.reset(climberEncoder.getPosition());
  }

  public void enablePID(boolean enabled){
    if(!PIDEnabled && enabled){
      hold();
    }
    PIDEnabled = enabled;
  }

  public boolean PIDEnabled(){
    return PIDEnabled;
  }

  public void climberManualControl(double speed){
    speed = MathUtil.applyDeadband(speed, 0.1);
    
    if(atLowerSoftLimit()) speed = Math.max(0, speed); 
    if(atUpperSoftLimit()) speed = Math.min(0, speed); 

    climberMotor.set(speed);
    enablePID(false);
  }

  public double getClimberPosition(){
    if(Robot.isSimulation()) return 0.0;
    return climberEncoder.getPosition();
  }

  private void resetClimberPostition(double position){
    climberEncoder.setPosition(position);
  }

  @Override 
  public void periodic(){
    if (PIDEnabled()){
      double output = climberPID.calculate(getClimberPosition());
      climberMotor.set(output);
    }
  }

  public void overrideLimits(boolean override){
    limitsEnabled = override;
  }

  public boolean getLimitsEnabled(){
    return limitsEnabled;
  }

  public void loadPreferences() {
    System.out.println("Loading climber PID values from preferences");
    climberPID.setP(Preferences.getDouble(ClimberConstants.kPkey, ClimberConstants.kPdefault));
    climberPID.setI(Preferences.getDouble(ClimberConstants.kIkey, ClimberConstants.kIdefault));
    climberPID.setD(Preferences.getDouble(ClimberConstants.kDkey, ClimberConstants.kDdefault));
    climberPID.setTolerance(Preferences.getDouble(ClimberConstants.PIDToleranceKey,
        ClimberConstants.kToleranceDefault));
    climberPID.setConstraints(new TrapezoidProfile.Constraints(
        Preferences.getDouble(ClimberConstants.climberMaxVelocityKey, ClimberConstants.climberMaxVelocityDefault),
        Preferences.getDouble(ClimberConstants.climberMaxAccelerationKey,
            ClimberConstants.climberMaxAccelerationDefault)));
    leftHookRetractedAngle = Preferences.getDouble(ClimberConstants.leftHookRetractedAnglekey,
        ClimberConstants.leftHookRetractedAngle);
    leftHookDeployedAngle = Preferences.getDouble(ClimberConstants.leftHookDeployedAnglekey,
        ClimberConstants.leftHookDeployedAngle);
    rightHookRetractedAngle = Preferences.getDouble(ClimberConstants.rightHookRetractedAnglekey,
        ClimberConstants.rightHookRetractedAngle);
    rightHookDeployedAngle = Preferences.getDouble(ClimberConstants.rightHookDeployedAnglekey,
        ClimberConstants.rightHookDeployedAngle);
    servoTime = Preferences.getDouble(ClimberConstants.climberServoTimeKey, ClimberConstants.climberServoTime);
    climberLowerSoftLimit = Preferences.getDouble(ClimberConstants.climberLowerSoftLimitKey,
        ClimberConstants.climberLowerSoftLimit);
    climberUpperSoftLimit = Preferences.getDouble(ClimberConstants.climberUpperSoftLimitKey,
        ClimberConstants.climberUpperSoftLimit);
  nudgeIncrement = Preferences.getDouble(ClimberConstants.climberNudgeIncrementKey, ClimberConstants.climberNudgeIncrement);
  }

  public void savePreferences() {
    System.out.println("Saving climber PID values to preferences");
    Preferences.setDouble(ClimberConstants.kPkey, climberPID.getP());
    Preferences.setDouble(ClimberConstants.kIkey, climberPID.getI());
    Preferences.setDouble(ClimberConstants.kDkey, climberPID.getD());
    Preferences.setDouble(ClimberConstants.PIDToleranceKey, climberPID.getPositionTolerance());
    Preferences.setDouble(ClimberConstants.leftHookRetractedAnglekey, leftHookRetractedAngle);
    Preferences.setDouble(ClimberConstants.leftHookDeployedAnglekey, leftHookDeployedAngle);
    Preferences.setDouble(ClimberConstants.rightHookRetractedAnglekey, rightHookRetractedAngle);
    Preferences.setDouble(ClimberConstants.rightHookDeployedAnglekey, rightHookDeployedAngle);
    Preferences.setDouble(ClimberConstants.climberServoTimeKey, servoTime);
    Preferences.setDouble(ClimberConstants.climberLowerSoftLimitKey, climberLowerSoftLimit);
    Preferences.setDouble(ClimberConstants.climberUpperSoftLimitKey, climberUpperSoftLimit);
  Preferences.setDouble(ClimberConstants.climberNudgeIncrementKey, nudgeIncrement);
  }

  //PID values, climber default speed, servo setpoints
  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addBooleanProperty("PID Enabled", () -> PIDEnabled(), (x)-> enablePID(x));
    builder.addDoubleProperty("kP", ()-> climberPID.getP(), (x)-> climberPID.setP(x));
    builder.addDoubleProperty("kI", ()-> climberPID.getI(), (x)-> climberPID.setI(x));
    builder.addDoubleProperty("kD", ()-> climberPID.getD(), (x)-> climberPID.setD(x));
    builder.addDoubleProperty("kTolerance", ()-> climberPID.getPositionTolerance(), (x)-> climberPID.setTolerance(x));
    builder.addDoubleProperty("left retracted angle", ()-> leftHookRetractedAngle, (x)-> leftHookRetractedAngle = x);
    builder.addDoubleProperty("right retracted angle", ()-> rightHookRetractedAngle, (x)-> rightHookRetractedAngle = x);
    builder.addDoubleProperty("left deployed angle", ()-> leftHookDeployedAngle, (x)-> leftHookDeployedAngle = x);
    builder.addDoubleProperty("right deployed angle", ()-> rightHookDeployedAngle, (x)-> rightHookDeployedAngle = x);
    builder.addDoubleProperty("servo time", ()-> servoTime, (x)-> servoTime = x);
    builder.addStringProperty("Hook Status", ()-> hookStatus(), null);
    builder.addDoubleProperty("Climber Position", ()-> getClimberPosition(), (x)-> resetClimberPostition(x));
    builder.addDoubleProperty("Climber Setpoint", ()-> climberSetpoint, (x)-> moveClimberPID(x));
    builder.addDoubleProperty("Motor Output", () -> climberMotor.getAppliedOutput(), null);
    builder.addDoubleProperty("lower limit", ()->climberLowerSoftLimit, (x)->climberLowerSoftLimit=x);
    builder.addDoubleProperty("upper limit", ()->climberUpperSoftLimit, (x)->climberUpperSoftLimit=x);
    builder.addDoubleProperty("nudge increment", ()-> nudgeIncrement, (x)-> nudgeIncrement = x);
    builder.addBooleanProperty("limits enabled", ()-> limitsEnabled, (x)-> overrideLimits(x));
    builder.addBooleanProperty("Save Prefs", () -> false, (x) -> { if(x){savePreferences();} });
  }


  public Command deployHooksCommand(){
    return new FunctionalCommand(
                                ()-> {deployHooks();},
                                ()-> {},
                                (x)-> {},
                                ()-> hooksReady(),
                                this                           
    );
  }

  public Command retractHooksCommand(){
    return new FunctionalCommand(
                                ()-> {retractHooks();},
                                ()-> {},
                                (x)-> {},
                                ()-> hooksReady(),
                                this                           
    );
  }

  
  public Command moveClimberToSetpointCommand(Double setpoint){
    return moveClimberToSetpointCommand(()->setpoint);
  }
  
  public Command moveClimberToSetpointCommand(DoubleSupplier setpoint){
    return new FunctionalCommand(
                                ()-> {moveClimberPID(setpoint.getAsDouble());},
                                ()-> {},
                                (x)-> {hold();},
                                ()-> atSetpoint(),
                                this                           
    );
  }

  public Command nudgeClimberPositionCommand(DoubleSupplier nudgeAmount){
    return new FunctionalCommand(
                                ()-> {},
                                ()-> {moveClimberPID(getClimberPosition() + (nudgeAmount.getAsDouble() * nudgeIncrement));},
                                (x)-> {hold();},
                                ()-> atSetpoint(),
                                this                           
    );
  }

  public Command climberManualControlCommand(DoubleSupplier speed){
    return new FunctionalCommand(
                                ()-> {climberManualControl(speed.getAsDouble());},
                                ()-> {},
                                (x)-> stopClimbers(),
                                ()-> false,
                                this                           
    );
  }

}