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

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;
import frc.utils.SwerveUtils;

public class IntakeArm extends SubsystemBase {

  private SparkMax m_armSparkMax;
  private DutyCycleEncoder m_ArmEncoder;
  private double m_armAngleOffset = IntakeConstants.armAngleOffset;
  public PIDController m_PidController;
  private ArmFeedforward m_FeedforwardLeft;
  private double kS = IntakeConstants.kS;
  private double kG = IntakeConstants.kG;
  private double kV = IntakeConstants.kV;
  private double kA = IntakeConstants.kA;
  private double kMaxArmAngle = IntakeConstants.MaxArmAngle;
  private double kMinArmAngle = IntakeConstants.MinArmAngle;
  private double kFloorPosition = IntakeConstants.kFloorPosition;
  private double kElevatorPosition = IntakeConstants.kElevatorPosition;
  private double kTroughPosition = IntakeConstants.kTroughPosition;
  private static final double kGearRatio = 100.0;
  private static final double kPositionConversionFactor = 360.0; // converts to degrees
  private static final double kVelocityConversionFactor = kPositionConversionFactor/ 60.0; // converts RPM to deg/sec
  private static final double kMaxVelocity = 100.0;

  private static final SparkMaxConfig leftSparkConfig = new SparkMaxConfig();
  static {
    leftSparkConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50);
}


  private LinearFilter m_velocityFilter;
  private double m_lastArmAngle;
  private double m_armVelocity = 0.0;
  private double pidLeft = 0;
  private double ffLeft = 0;
  private double m_goal = kMaxArmAngle;

  private double kPositionTolerance = IntakeConstants.kPositionTolerance;
    
      /** Creates a new IntakeArm. */
      public IntakeArm() {
    
        m_armSparkMax = new SparkMax(Constants.CanIds.intakeArmMotorID, MotorType.kBrushed);
        m_armSparkMax.configure(leftSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
        m_ArmEncoder = new DutyCycleEncoder(Constants.DigitalIO.kIntakeArmEncoderDIO);
        m_ArmEncoder.setInverted(true);
      
  
        m_PidController = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
        m_PidController.reset();
        resetFeedForward();
        m_velocityFilter = LinearFilter.singlePoleIIR(0.1, Robot.kDefaultPeriod);
        m_lastArmAngle = getArmAngleDegrees();
        m_goal = getArmAngleDegrees();
        stop();
      }
    
      private void resetFeedForward() {
        m_FeedforwardLeft = new ArmFeedforward(kS, kG, kV, kA);
      }
    
      @Override
      public void periodic() {
        // This method will be called once per scheduler run
        double newAngle = getArmAngleDegrees();
        double vel = (newAngle - m_lastArmAngle) / Robot.kDefaultPeriod;
        m_armVelocity = m_velocityFilter.calculate(vel);
        m_lastArmAngle = newAngle;
    
        double setpoint = SwerveUtils.StepTowards(m_PidController.getSetpoint(), m_goal, kMaxVelocity * Robot.kDefaultPeriod);
        m_PidController.setSetpoint(setpoint);
        pidLeft = m_PidController.calculate(newAngle);
        ffLeft = m_FeedforwardLeft.calculate(Math.toRadians(getArmAngleDegrees()), 0.0);

        if(!DriverStation.isTest() && DriverStation.isEnabled()){
          m_armSparkMax.set((pidLeft + ffLeft));
        }
    }
  
    public void setGoal(double angle) {
      m_goal = MathUtil.clamp(angle, kMinArmAngle, kMaxArmAngle);
    }
  
    public boolean atGoal(){
      return MathUtil.isNear(m_goal, getArmAngleDegrees(), kPositionTolerance);
    }
  
    public void changeGoal(double delta){
      double p = getPositionGoal();
      setGoal(p + delta);
    }
  
    public double getPositionGoal() {
      return m_goal;
    }
  
    public double getArmAngleDegrees() {
      double value = (m_ArmEncoder.get() * kPositionConversionFactor) + m_armAngleOffset;
      if(value > 180.0){
        value -= 360.0;
      }

      return value;
    }
  
    public double getArmAngleVelocity() {
      return m_armVelocity;
    }
  
    public void floorPosition(){ setGoal(kFloorPosition); }
    public void troughPosition(){ setGoal(kTroughPosition); }
    public void elevatorPosition(){ setGoal(kElevatorPosition); }

    public Command setPositionCommand(double position){
      return new FunctionalCommand(
                            ()->setGoal(position),
                            ()->{},
                            (x)->{},
                            ()->atGoal(),
                            this
                            );
    }

    public Command changePositionCommand(double delta){
      return new InstantCommand(()->changeGoal(delta));
    }
  
    public Command testCommand(double speed){
      return new FunctionalCommand(
                            ()->{m_armSparkMax.set(speed);},
                            ()->{},
                            (x)->{m_armSparkMax.set(0.0);},
                            ()->false,
                            this
                            );
    }

    public String getPositionLabel(){
      double pos = getArmAngleDegrees();
      if(MathUtil.isNear(kFloorPosition, pos, kPositionTolerance)){
      return "Floor";
    }else if(MathUtil.isNear(kTroughPosition, pos, kPositionTolerance)){
      return "Trough";
    }else if(MathUtil.isNear(kElevatorPosition, pos, kPositionTolerance)){
      return "Elevator";
    }else{
      return String.format("%.1f", pos);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("ArmAngle", this::getArmAngleDegrees, null);
    builder.addDoubleProperty("Goal", this::getPositionGoal, this::setGoal);
    builder.addDoubleProperty("ArmAnglOffset", () -> m_armAngleOffset, (x) -> {
      m_armAngleOffset = x;
    });
    builder.addDoubleProperty("kS", () -> kS, (x) -> {
      kS = x;
      resetFeedForward();
    });
    builder.addDoubleProperty("kG", () -> kG, (x) -> {
      kG = x;
      resetFeedForward();
    });
    builder.addDoubleProperty("kV", () -> kV, (x) -> {
      kV = x;
      resetFeedForward();
    });
    builder.addDoubleProperty("kA", () -> kA, (x) -> {
      kA = x;
      resetFeedForward();
    });
    builder.addDoubleProperty("kMinArmAngle", () -> kMinArmAngle, (x) -> {
      kMinArmAngle = x;
    });
    builder.addDoubleProperty("kMaxArmAngle", () -> kMaxArmAngle, (x) -> {
      kMaxArmAngle = x;
    });
    builder.addStringProperty("pid", ()->String.format("%.2f", pidLeft), null);
    builder.addStringProperty("ffLeft", ()->String.format("%.2f", ffLeft), null);
    builder.addDoubleProperty("motorOutput", ()->m_armSparkMax.getAppliedOutput(), null);
    builder.addStringProperty("ArmAngleLabel", this::getPositionLabel, null);
  }


  public void stop() {
    m_goal = getArmAngleDegrees();
    m_PidController.setSetpoint(m_goal);
    m_armSparkMax.set(0.0);  
  }
}