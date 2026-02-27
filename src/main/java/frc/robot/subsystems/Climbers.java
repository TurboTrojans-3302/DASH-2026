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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Climbers extends SubsystemBase{

  SparkMax climberMotor;
  PIDController climberPID;
  double kP = Constants.ClimberConstants.kP;
  double kI = Constants.ClimberConstants.kI;
  double kD = Constants.ClimberConstants.kD;
  double kTolerance = Constants.ClimberConstants.kTolerance;
  Servo servoInnerLeft;
  Servo servoInnerRight;
  Servo servoOuterLeft;
  Servo servoOuterRight;
  double servoInnerLeftSetpoint = 0.0;
  double servoInnerRightSetpoint = 0.0;
  double servoOuterLeftSetpoint = 0.0;
  double servoOuterRightSetpoint = 0.0;
  double servoAngleTolerance = 1.0;
  DigitalInput lowerLimitSwitch;
  double hookRetractedAngle = Constants.ClimberConstants.hookRetractedAngle;
  double hookDeployedAngle = Constants.ClimberConstants.hookDeployedAngle;
  double hookEngagedAngle = Constants.ClimberConstants.hookEngagedAngle;

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

  public void RetractInnerHooks(){
    setInnerServosPosition(hookRetractedAngle);
  }

  public void RetractOuterHooks(){
    setOuterServosPosition(hookRetractedAngle);
  }

  public void DeployInnerHooks(){
    setInnerServosPosition(hookDeployedAngle);
  }

  public void DeployOuterHooks(){
    setOuterServosPosition(hookDeployedAngle);
  }

  public void EngageInnerHooks(){
    setInnerServosPosition(hookEngagedAngle);
  }

  public void EngageOuterHooks(){
    setOuterServosPosition(hookEngagedAngle);
  }

  public Boolean InnerHooksAtSetpoint(){
    //surely there is a neater way to do this
    //there has to be some sort of math function that can check if a value is within a certain range but I havent found it yet
    return (((servoInnerLeftSetpoint - servoAngleTolerance) <= (servoInnerLeft.getAngle())) 
    && (servoInnerLeft.getAngle()) <= (servoInnerLeftSetpoint + servoAngleTolerance)) 
    
    &&

    (((servoInnerLeftSetpoint - servoAngleTolerance) <= (servoInnerLeft.getAngle())) 
    && (servoInnerLeft.getAngle()) <= (servoInnerLeftSetpoint + servoAngleTolerance));
  } 

  public Boolean OuterHooksAtSetpoint(){
    
    return((((servoInnerLeftSetpoint - servoAngleTolerance) <= (servoInnerLeft.getAngle())) 
    && (servoInnerLeft.getAngle()) <= (servoInnerLeftSetpoint + servoAngleTolerance)) 
    
    &&

    (((servoInnerLeftSetpoint - servoAngleTolerance) <= (servoInnerLeft.getAngle())) 
    && (servoInnerLeft.getAngle()) <= (servoInnerLeftSetpoint + servoAngleTolerance)));
  } 


  public void setClimberPosition(double setpoint){
    climberMotor.set(
      climberPID.calculate(climberEncoder.getPosition(), setpoint)
    );
  }


}