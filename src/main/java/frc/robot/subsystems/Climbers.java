// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.Math;
import java.time.temporal.ValueRange;

import edu.wpi.first.math.MathUtil;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Climbers extends SubsystemBase{

  SparkMax climberMotor;
  double climberSpeed = Constants.ClimberConstants.climberDefaultSpeed;
  PIDController climberPID;
  double kP = Constants.ClimberConstants.kPdefault;
  double kI = Constants.ClimberConstants.kIdefault;
  double kD = Constants.ClimberConstants.kDdefault;
  double kTolerance = Constants.ClimberConstants.kToleranceDefault;
  Servo servoInnerLeft;
  Servo servoInnerRight;
  Servo servoOuterLeft;
  Servo servoOuterRight;
  boolean innerHooksAtSetpoint = true;
  boolean outerHooksAtSetpoint = true;
  double servoInnerLeftSetpoint = 0.0;
  double servoInnerRightSetpoint = 0.0;
  double servoOuterLeftSetpoint = 0.0;
  double servoOuterRightSetpoint = 0.0;
  double servoAngleTolerance = 1.0;
  Timer servoTimer;
  DigitalInput lowerLimitSwitch;
  double hookRetractedAngle = Constants.ClimberConstants.hookRetractedAngle;
  double hookDeployedAngle = Constants.ClimberConstants.hookDeployedAngle;
  

  RelativeEncoder climberEncoder;




  public Climbers() {

    SparkMax climberMotor = new SparkMax(Constants.CanIds.kClimbMotor, MotorType.kBrushless);
     climberMotor.configure(new SparkMaxConfig().inverted(false)
        .idleMode(IdleMode.kBrake),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
    climberPID = new PIDController(kP,kI,kD);
    climberPID.setTolerance(kTolerance);
    climberPID.reset();
    climberEncoder = climberMotor.getEncoder();
    climberEncoder.setPosition(0.0);

    servoInnerLeft = new Servo(Constants.PWMChannels.PWMServoInnerLeft);
    servoInnerRight = new Servo(Constants.PWMChannels.PWMServoInnerRight);
    servoOuterLeft = new Servo(Constants.PWMChannels.PWMServoOuterLeft);
    servoOuterRight = new Servo(Constants.PWMChannels.PWMServoOuterRight);

    

    DigitalInput lowerLimitSwitch = new DigitalInput(Constants.DigitalIO.kClimberLimitSwitch);

    servoTimer.restart();
    
    
  }

  public void resetServoTimer(){
    servoTimer.reset();
    servoTimer.start();
  }

  public boolean waitForServo(){
    return servoTimer.advanceIfElapsed(0.5);
  }
  public boolean atLowerLimit(){
    return lowerLimitSwitch.get();
  }

  public void setInnerServosPosition(double angle){
    servoInnerLeft.setAngle(angle);
    servoInnerRight.setAngle(angle);
    servoInnerLeftSetpoint = servoInnerRightSetpoint = angle;

  }

  public void setOuterServosPosition(double angle){
    servoOuterLeft.setAngle(angle);
    servoOuterRight.setAngle(angle);
    servoOuterLeftSetpoint = servoOuterRightSetpoint = angle;

  }

  // public void RetractInnerHooks(){
  //   setInnerServosPosition(hookRetractedAngle);
  // 
  public Command RetractInnerHooks(){
    return new FunctionalCommand(
                                ()-> {setInnerServosPosition(hookRetractedAngle);
                                      resetServoTimer();
                                      innerHooksAtSetpoint = false;},
                                ()-> {},
                                (x)-> {innerHooksAtSetpoint = true;},
                                ()-> waitForServo(),
                                this                           
    );
  }

  

  public Command RetractOuterHooks(){
    return new FunctionalCommand(
                                ()-> {setOuterServosPosition(hookRetractedAngle);
                                      resetServoTimer();
                                      outerHooksAtSetpoint = false;},
                                ()-> {},
                                (x)-> {outerHooksAtSetpoint = true;},
                                ()-> waitForServo(),
                                this                           
    );
  }

  public Command DeployInnerHooks(){
    return new FunctionalCommand(
                                ()-> {setInnerServosPosition(hookDeployedAngle);
                                      resetServoTimer();
                                      innerHooksAtSetpoint = false;},
                                ()-> {},
                                (x)-> {innerHooksAtSetpoint = true;},
                                ()-> waitForServo(),
                                this                           
    );
  }

  public Command DeployOuterHooks(){
    return new FunctionalCommand(
                                ()-> {setOuterServosPosition(hookDeployedAngle);
                                      resetServoTimer();
                                      outerHooksAtSetpoint = false;},
                                ()-> {},
                                (x)-> {outerHooksAtSetpoint = true;},
                                ()-> waitForServo(),
                                this                           
    );
  }

  public Boolean InnerHooksAtSetpoint(){
   return innerHooksAtSetpoint;
  } 

  public Boolean OuterHooksAtSetpoint(){
    return outerHooksAtSetpoint;
  }

  public void setClimberPosition(double setpoint){
    climberMotor.set(
      climberPID.calculate(climberEncoder.getPosition(), setpoint)
    );
  }

  public boolean ClimberAtSetpoint(){
    return climberPID.atSetpoint();
  }

  public void loadPreferences() {
    if (Preferences.containsKey(Constants.ClimberConstants.kPkey)) {
      System.out.println("Loading climber PID values from preferences");
      climberPID.setP(Preferences.getDouble(Constants.ClimberConstants.kPkey, Constants.ClimberConstants.kPdefault));
      climberPID.setI(Preferences.getDouble(Constants.ClimberConstants.kIkey, Constants.ClimberConstants.kIdefault));
      climberPID.setD(Preferences.getDouble(Constants.ClimberConstants.kDkey, Constants.ClimberConstants.kDdefault));
      climberPID.setTolerance(Preferences.getDouble(Constants.ClimberConstants.PIDToleranceKey,
          Constants.ClimberConstants.kToleranceDefault));
      climberSpeed = Preferences.getDouble(Constants.ShooterConstants.feederSpeedKey,
          Constants.ShooterConstants.feederSpeedDefault); // TODO here
    } else {
      System.out.println("No climber prefs found. Using default values");
    }
  }

  public void savePreferences() {
    System.out.println("Saving climber PID values to preferences");
    Preferences.setDouble(Constants.ClimberConstants.kPkey, climberPID.getP());
    Preferences.setDouble(Constants.ClimberConstants.kIkey, climberPID.getI());
    Preferences.setDouble(Constants.ClimberConstants.kDkey, climberPID.getD());
    Preferences.setDouble(Constants.ClimberConstants.PIDToleranceKey, climberPID.getErrorTolerance());
    //TODO add climber speed here

  }


}