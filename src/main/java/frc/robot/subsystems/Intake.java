// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

 /*  private double upSpeed = Constants.IntakeConstants.upSpeed;
  private double downSpeed = Constants.IntakeConstants.downSpeed;
  private double upperLoadSpeed = Constants.IntakeConstants.upperLoadSpeed;

  private static final SparkMaxConfig sparkConfig = new SparkMaxConfig();
  static {
    sparkConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(40);
  }
*/
  /** Creates a new Intake. */
 // private SparkMax m_intakeMotor;
 // private DigitalInput m_objectDetectSwitch;
  //private DigitalInput m_upperObjectDetectSwitch;

  public Intake() {
  
  }

 /*  public void stop(){
    
  }
  */
 /*  public double getIntakeSpeed(){
    
  }
*/
  /*
   * negative speed pulls in, positive speed pushes out
   */
 /*  public void setIntakeSpeed(double speed){
    m_intakeMotor.set(MathUtil.clamp(speed, Constants.IntakeConstants.intakeSpeedMin, Constants.IntakeConstants.intakeSpeedMax));
  }
*/
 /*  public boolean objectDetected(){
    return !m_objectDetectSwitch.get();
  }
*/
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   /*   if (objectDetected()){
      setIntakeSpeed(Math.max(getIntakeSpeed(), 0.0));
    }
*/
  }

 /* public void down(){
    setIntakeSpeed(downSpeed);
  }

  public void up(){
    setIntakeSpeed(upSpeed);
  }
*/
 /*  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Intake Speed", this::getIntakeSpeed, this::setIntakeSpeed);
    builder.addDoubleProperty("downSpeed", ()->downSpeed, (x)->downSpeed = x);
    builder.addDoubleProperty("upSpeed", ()->upSpeed, (x)->upSpeed = x);
    builder.addDoubleProperty("motor output", ()->m_intakeMotor.getAppliedOutput(), null);
  }
*/
  

  
}