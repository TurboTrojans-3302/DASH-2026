// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.PWMChannels;


public class Climber extends SubsystemBase{

  private SparkMax climberMotor;
  private double climberSpeed = 0;
  private double climberSetpoint = 0;
  private PIDController climberPID;
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

  private RelativeEncoder climberEncoder;




  public Climber() {

    loadPreferences();

    climberMotor = new SparkMax(Constants.CanIds.kClimbMotor, MotorType.kBrushless);
    climberMotor.configure(new SparkMaxConfig().inverted(false)
        .idleMode(IdleMode.kBrake),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
    climberPID = new PIDController(ClimberConstants.kPdefault, ClimberConstants.kIdefault, ClimberConstants.kDdefault);
    climberPID.setTolerance(ClimberConstants.kToleranceDefault);
    climberPID.reset();
    climberEncoder = climberMotor.getEncoder();
    climberEncoder.setPosition(0.0);

    servoLeft = new Servo(PWMChannels.PWMServoLeft);
    servoRight = new Servo(PWMChannels.PWMServoRight);

    lowerLimitSwitch = new DigitalInput(Constants.DigitalIO.kClimberLimitSwitch);
    servoTimer = new Timer();
  
    retractHooks();
  }


  public boolean atLowerSoftLimit(){
    return getClimberPosition() <= climberLowerSoftLimit;
  }

  public boolean atUpperSoftLimit(){
    return getClimberPosition() >= climberUpperSoftLimit;
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
    return servoTimer.hasElapsed(climberSetpoint);
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
  }

  public boolean atSetpoint(){
    return climberPID.atSetpoint();
  }

  public double getClimberManualSpeed(){
    return climberSpeed;
  }

  public void stopClimbers(){
    climberMotor.stopMotor();
  }

  public void hold(){
    moveClimberPID(climberEncoder.getPosition());
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

  public void climberManualControl(double speed, boolean overrideLimits){
    if(!overrideLimits){
      if(atLowerSoftLimit()) speed = Math.max(0, speed); 
      if(atUpperSoftLimit()) speed = Math.min(0, speed); 
    }

    climberMotor.set(speed);
    enablePID(false);
  }

  public double getClimberPosition(){
    return climberEncoder.getPosition();
  }

  private void resetClimberPostition(double position){
    climberEncoder.setPosition(position);
  }

  @Override 
  public void periodic(){
    if (PIDEnabled()){
      double output = climberPID.calculate(getClimberPosition(), climberSetpoint);
      climberMotor.set(output);
    }
  }


  public void loadPreferences() {
      System.out.println("Loading climber PID values from preferences");
      climberPID.setP(Preferences.getDouble(ClimberConstants.kPkey, ClimberConstants.kPdefault));
      climberPID.setI(Preferences.getDouble(ClimberConstants.kIkey, ClimberConstants.kIdefault));
      climberPID.setD(Preferences.getDouble(ClimberConstants.kDkey, ClimberConstants.kDdefault));
      climberPID.setTolerance(Preferences.getDouble(ClimberConstants.PIDToleranceKey,
          ClimberConstants.kToleranceDefault));
      climberSpeed = Preferences.getDouble(ClimberConstants.climberSpeedKey,
          ClimberConstants.climberDefaultSpeed);
      leftHookRetractedAngle = Preferences.getDouble(ClimberConstants.leftHookRetractedAnglekey, ClimberConstants.leftHookRetractedAngle);
      leftHookDeployedAngle = Preferences.getDouble(ClimberConstants.leftHookDeployedAnglekey, ClimberConstants.leftHookDeployedAngle);
      rightHookRetractedAngle = Preferences.getDouble(ClimberConstants.rightHookRetractedAnglekey, ClimberConstants.rightHookRetractedAngle);
      rightHookDeployedAngle = Preferences.getDouble(ClimberConstants.rightHookDeployedAnglekey, ClimberConstants.rightHookDeployedAngle);
      servoTime = Preferences.getDouble(ClimberConstants.climberServoTimeKey, ClimberConstants.climberServoTime);
      climberLowerSoftLimit = Preferences.getDouble(ClimberConstants.climberLowerSoftLimitKey, ClimberConstants.climberLowerSoftLimit);
      climberUpperSoftLimit = Preferences.getDouble(ClimberConstants.climberUpperSoftLimitKey, ClimberConstants.climberUpperSoftLimit);
  }

  public void savePreferences() {
    System.out.println("Saving climber PID values to preferences");
    Preferences.setDouble(ClimberConstants.kPkey, climberPID.getP());
    Preferences.setDouble(ClimberConstants.kIkey, climberPID.getI());
    Preferences.setDouble(ClimberConstants.kDkey, climberPID.getD());
    Preferences.setDouble(ClimberConstants.PIDToleranceKey, climberPID.getErrorTolerance());
    Preferences.setDouble(ClimberConstants.climberSpeedKey, climberSpeed);
    Preferences.setDouble(ClimberConstants.leftHookRetractedAnglekey, leftHookRetractedAngle);
    Preferences.setDouble(ClimberConstants.leftHookDeployedAnglekey, leftHookDeployedAngle);
    Preferences.setDouble(ClimberConstants.rightHookRetractedAnglekey, rightHookRetractedAngle);
    Preferences.setDouble(ClimberConstants.rightHookDeployedAnglekey, rightHookDeployedAngle);
    Preferences.setDouble(ClimberConstants.climberServoTimeKey, servoTime);
    Preferences.setDouble(ClimberConstants.climberLowerSoftLimitKey, climberLowerSoftLimit);
    Preferences.setDouble(ClimberConstants.climberUpperSoftLimitKey, climberUpperSoftLimit);
  }

  //PID values, climber default speed, servo setpoints
  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addBooleanProperty("PID Enabled", () -> PIDEnabled(), (x)-> enablePID(x));
    builder.addDoubleProperty("kP", ()-> climberPID.getP(), (x)-> climberPID.setP(x));
    builder.addDoubleProperty("kI", ()-> climberPID.getI(), (x)-> climberPID.setI(x));
    builder.addDoubleProperty("kD", ()-> climberPID.getD(), (x)-> climberPID.setD(x));
    builder.addDoubleProperty("kTolerance", ()-> climberPID.getErrorTolerance(), (x)-> climberPID.setTolerance(x));
    builder.addDoubleProperty("climber manual speed", ()-> getClimberManualSpeed(), (x)-> climberSpeed = x);
    builder.addDoubleProperty("left retracted angle", ()-> leftHookRetractedAngle, (x)-> leftHookRetractedAngle = x);
    builder.addDoubleProperty("right retracted angle", ()-> rightHookRetractedAngle, (x)-> rightHookRetractedAngle = x);
    builder.addDoubleProperty("left deployed angle", ()-> leftHookDeployedAngle, (x)-> leftHookDeployedAngle = x);
    builder.addDoubleProperty("right deployed angle", ()-> rightHookDeployedAngle, (x)-> rightHookDeployedAngle = x);
    builder.addDoubleProperty("servo time", ()-> servoTime, (x)-> servoTime = x);
    builder.addStringProperty("Hook Status", ()-> hookStatus(), null);
    builder.addDoubleProperty("Climber Position", ()-> getClimberPosition(), (x)-> resetClimberPostition(x));
    builder.addDoubleProperty("Motor Output", () -> climberMotor.getAppliedOutput(), null);
    builder.addDoubleProperty("lower limit", ()->climberLowerSoftLimit, (x)->climberLowerSoftLimit=x);
    builder.addDoubleProperty("upper limit", ()->climberUpperSoftLimit, (x)->climberUpperSoftLimit=x);
  }


  public Command DeployHooks(){
    return new FunctionalCommand(
                                ()-> {deployHooks();},
                                ()-> {},
                                (x)-> {},
                                ()-> hooksReady(),
                                this                           
    );
  }

  public Command RetractHooks(){
    return new FunctionalCommand(
                                ()-> {retractHooks();},
                                ()-> {},
                                (x)-> {},
                                ()-> hooksReady(),
                                this                           
    );
  }

  public Command MoveClimberToSetpoint(DoubleSupplier setpoint){
    return new FunctionalCommand(
                                ()-> {moveClimberPID(setpoint.getAsDouble());},
                                ()-> {},
                                (x)-> {hold();},
                                ()-> atSetpoint(),
                                this                           
    );
  }

  public Command NudgeClimberPosition(DoubleSupplier nudgeAmount){
    return new FunctionalCommand(
                                ()-> {},
                                ()-> {moveClimberPID(getClimberPosition() + nudgeAmount.getAsDouble());},
                                (x)-> {hold();},
                                ()-> atSetpoint(),
                                this                           
    );
  }

  public Command ClimberManualControlCommand(DoubleSupplier speed, BooleanSupplier overrideLimits){
    return new FunctionalCommand(
                                ()-> {climberManualControl(speed.getAsDouble(), overrideLimits.getAsBoolean());},
                                ()-> {},
                                (x)-> stopClimbers(),
                                ()-> false,
                                this                           
    );
  }
}