/*
 * TODO intakeArm bringup steps
 * confirm positive speed == arm go up
 * confirm L and R motors are going the same way
 * confirm arm-go-up means and increase in encoder value
 * confirm encoder value is degrees, or recalculate kPositionConversionFactor
 * adjust m_armAngleOffset so that 0deg is maximum torque position 
 * Calibrate kMaxArmAngle and kMinArmAngle
 * Calibrate kFloorPosition, kElevatorPosition and kTroughPosition?
 * Set kG
 * Tune PID (keep profile constraints very high)
 * Tune profile constraints
 * 
 */

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeArm extends SubsystemBase {

    
      /** Creates a new IntakeArm. */
      public IntakeArm() {
    
        
      }
    
  
      @Override
      public void periodic() {
        // This method will be called once per scheduler run
        }
    
    
  
   
  
    
  

  @Override
  public void initSendable(SendableBuilder builder) {
   


 
}
}