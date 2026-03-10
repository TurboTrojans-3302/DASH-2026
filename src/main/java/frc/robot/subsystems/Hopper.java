package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;
import frc.utils.PrefValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Hopper subsystem controls two Neo550 motors to expand or retract the hopper.
 */
public class Hopper extends SubsystemBase {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final RelativeEncoder leftEncoder, rightEncoder;
    private final DigitalInput leftContractedLimitSwitch = new DigitalInput(Constants.DigitalIO.kHopperLeftContractedLimitSwitchDio);
    private final DigitalInput rightContractedLimitSwitch = new DigitalInput(Constants.DigitalIO.kHopperRightContractedLimitSwitchDio);
    private final PrefValue<Double> softMax         = new PrefValue<>("hopperMaxPosition",      220.0, this);
    private final PrefValue<Double> softMin         = new PrefValue<>("hopperMinPosition",      3.0,   this);
    private final PrefValue<Double> kP              = new PrefValue<>("hopperKp",               0.02,  this);
    private final PrefValue<Double> kI              = new PrefValue<>("hopperKi",               0.0,   this);
    private final PrefValue<Double> kD              = new PrefValue<>("hopperKd",               0.0,   this);
    private final PrefValue<Double> posTolerance    = new PrefValue<>("hopperPosTolerance",     1.0,   this);
    private final PrefValue<Double> kG              = new PrefValue<>("hopperKg",               0.0,   this);
    private final PrefValue<Double> maxVelocity     = new PrefValue<>("hopperMaxVelocity",      80.0,  this);
    private final PrefValue<Double> maxAcceleration = new PrefValue<>("hopperMaxAcceleration",  50.0,  this);
    private ElevatorFeedforward feedforward = new ElevatorFeedforward(0, HopperConstants.kGdefault, 0);
    private boolean PIDEnabled = false;
    private double positionSetpoint = 0.0;

    private final ProfiledPIDController leftPID  = new ProfiledPIDController(
            HopperConstants.kPdefault, HopperConstants.kIdefault, HopperConstants.kDdefault,
            new TrapezoidProfile.Constraints(HopperConstants.maxVelocityDefault, HopperConstants.maxAccelerationDefault));
    private final ProfiledPIDController rightPID = new ProfiledPIDController(
            HopperConstants.kPdefault, HopperConstants.kIdefault, HopperConstants.kDdefault,
            new TrapezoidProfile.Constraints(HopperConstants.maxVelocityDefault, HopperConstants.maxAccelerationDefault));


    public Hopper(int leftMotorID, int rightMotorID) {
        leftMotor = new SparkMax(leftMotorID, MotorType.kBrushless);
        rightMotor = new SparkMax(rightMotorID, MotorType.kBrushless);

        configureSparkMaxes();

        leftMotor.set(0);
        leftEncoder = leftMotor.getEncoder();

        rightMotor.set(0);
        rightEncoder = rightMotor.getEncoder();

        // Wire onChange callbacks so dashboard edits take effect immediately
        kP.onChange(x -> applyPIDGains());
        kI.onChange(x -> applyPIDGains());
        kD.onChange(x -> applyPIDGains());
        kG.onChange(x -> applyPIDGains());
        posTolerance.onChange(x -> applyPIDGains());
        maxVelocity.onChange(x -> applyPIDGains());
        maxAcceleration.onChange(x -> applyPIDGains());

        applyPIDGains();
    }

    private void configureSparkMaxes() {
        SparkMaxConfig leftSparkConfig = new SparkMaxConfig();
        leftSparkConfig.apply(SparkMaxConfig.Presets.REV_NEO_550);
        leftSparkConfig.idleMode(IdleMode.kBrake);
        leftSparkConfig.inverted(true);

        SparkMaxConfig rightSparkConfig = new SparkMaxConfig().apply(leftSparkConfig);

        leftMotor.configure(leftSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Convenience constructor that uses CAN IDs from
     * {@link frc.robot.Constants.HopperConstants}.
     */
    public Hopper() {
        this(Constants.CanIds.kHopperLeftMotorCanId, Constants.CanIds.kHopperRightMotorCanId);
    }

    public boolean isEmpty() {
        // Placeholder for actual sensor check to determine if hopper is empty
        // This could be implemented with a limit switch, ultrasonic sensor, etc.
        return false; // Assume not empty for now
    }

    public void move(double speedLeft, double speedRight) {
        if (isPIDEnabled()) {
            // Do nothing
        } else {
            double motorSpeedLeft = MathUtil.clamp(speedLeft, -1.0, 1.0);
            double motorSpeedRight = MathUtil.clamp(speedRight, -1.0, 1.0);
            setMotorPctOutput(motorSpeedLeft, motorSpeedRight);
        }
    }

    public void setPosition(double pos) {
        if(isPIDEnabled()) {
            positionSetpoint = MathUtil.clamp(pos, softMin.get(), softMax.get());
            leftPID.setGoal(positionSetpoint);
            rightPID.setGoal(positionSetpoint);
        }else{
            DriverStation.reportWarning("Can't set hopper position while PID is disabled!", false);
        }
    }

    public boolean isPIDEnabled() {
        return PIDEnabled;
    }

    public double getPosition() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
    }

    public void setPIDEnabled(boolean enabled) {
        if (enabled && !PIDEnabled) { //turning on PID
            PIDEnabled = enabled;
            // Snap setpoint to current average position and reset controllers
            double currentPos = getPosition();
            setPosition(currentPos);
            leftPID.reset(currentPos);
            rightPID.reset(currentPos);
        }

        if(!enabled && PIDEnabled){ //turning off PID
            stop(); // stop motors when disabling PID
        }

        PIDEnabled = enabled;
    }

    /**
     * Stop both hopper motors.
     */
    public void stop() {
        setMotorPctOutput(0, 0);
    }

    public void hold() {
        if (isPIDEnabled()) {
            setPosition(getPosition()); // re-apply current setpoint to hold position
        } else {
            stop();
        }
    }

    private void setMotorPctOutput(double leftSpeed, double rightSpeed) {
        leftSpeed  = MathUtil.clamp(leftSpeed,  -1.0, 1.0);
        rightSpeed = MathUtil.clamp(rightSpeed, -1.0, 1.0);
        if(atSoftMaxL()) leftSpeed = Math.min(0, leftSpeed); // if at max, only allow retracting (negative speed)
        if(atSoftMaxR()) rightSpeed = Math.min(0, rightSpeed); // if at max, only allow retracting (negative speed)
        if(atSoftMinL()) leftSpeed = Math.max(0, leftSpeed); // if at min, only allow extending (positive speed)
        if(atSoftMinR()) rightSpeed = Math.max(0, rightSpeed); // if at min, only allow extending (positive speed)

        leftMotor.set(leftSpeed);
        rightMotor.set(rightSpeed);
    }

    @Override
    public void periodic() {
        if (leftHardLimit())  { leftEncoder.setPosition(0); }
        if (rightHardLimit()) { rightEncoder.setPosition(0); }

        if (isPIDEnabled()) {
            double ffOutput = feedforward.calculate(0); // static gravity compensation (velocity = 0)
            double leftOutput  = leftPID.calculate(leftEncoder.getPosition()) + ffOutput;
            double rightOutput = rightPID.calculate(rightEncoder.getPosition()) + ffOutput;
            setMotorPctOutput(leftOutput, rightOutput);
        }
    }

    public boolean atSoftMaxL() {
        return leftEncoder.getPosition() >= softMax.get();
    }

    public boolean atSoftMaxR() {
        return rightEncoder.getPosition() >= softMax.get();
    }

    public boolean atMaxPosition() {
        return atSoftMaxL() && atSoftMaxR();
    }

    public boolean atSoftMinL() {
        return leftEncoder.getPosition() <= softMin.get();
    }

    public boolean atSoftMinR() {
        return rightEncoder.getPosition() <= softMin.get();
    }

    public boolean atMinPosition() {
        return atSoftMinL() && atSoftMinR();
    }

    public Command stopCommand() {
        Command cmd = new InstantCommand(this::stop, this);
        cmd.setName("stopCommand");
        return cmd;
    }

    public Command setPositionCommand(DoubleSupplier targetPosition) {
        FunctionalCommand cmd = new FunctionalCommand(
            () -> { setPosition(targetPosition.getAsDouble()); },
            () -> {}, // periodic() handles the output
            (interrupted) -> { if (interrupted) hold(); }, // stop on interrupt
            () -> (leftPID.atGoal() && rightPID.atGoal()) || !isPIDEnabled(), // end command when at position or if PID is disabled
            this);
        cmd.setName("setPositionCommand");
        return cmd;
    }

    public Command expandCommand() {
        Command cmd = setPositionCommand(() -> softMax.get());
        cmd.setName("expandCommand");
        return cmd;
    }

    public Command retractCommand() {
        Command cmd = setPositionCommand(() -> softMin.get());
        cmd.setName("retractCommand");
        return cmd;
    }

        public Command manualMoveCommand(DoubleSupplier speedSupplierLeft, DoubleSupplier speedSupplierRight) {
        Command cmd = new FunctionalCommand(
            () -> {}, // no init
            () -> move(speedSupplierLeft.getAsDouble(), speedSupplierRight.getAsDouble()), // call move() with supplier value
            (interrupted) -> { stop(); }, // stop on end or interrupt
            () -> false, // never end on its own
            this);
        cmd.setName("manualMoveCommand");
        return cmd;
    }

    public Command nudgeCommand(double incrementPerLoop) {
        Command cmd = new FunctionalCommand(
            () -> {}, // no init needed
            () -> setPosition(positionSetpoint + incrementPerLoop), // advance setpoint each loop
            (interrupted) -> hold(), // hold position on release
            () -> false, // whileTrue in RobotContainer handles termination
            this);
        cmd.setName("nudgeCommand");
        return cmd;
    }

    /** Apply current kP/kI/kD/kG/tolerance/constraints to both PID controllers and feedforward. */
    private void applyPIDGains() {
        leftPID.setPID(kP.get(), kI.get(), kD.get());
        rightPID.setPID(kP.get(), kI.get(), kD.get());
        leftPID.setTolerance(posTolerance.get());
        rightPID.setTolerance(posTolerance.get());
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get());
        leftPID.setConstraints(constraints);
        rightPID.setConstraints(constraints);
        feedforward = new ElevatorFeedforward(0, kG.get(), 0);
    }

    public boolean leftHardLimit() {
        return !leftContractedLimitSwitch.get();
    }

    public boolean rightHardLimit() {
        return !rightContractedLimitSwitch.get();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        PrefValue.addAllBuilderProperties(this, builder);
        builder.addDoubleProperty("Left Position", () -> leftEncoder.getPosition(), (x)->leftEncoder.setPosition(x));
        builder.addDoubleProperty("Right Position", () -> rightEncoder.getPosition(), (x)->rightEncoder.setPosition(x));
        builder.addDoubleProperty("Position Setpoint", () -> positionSetpoint, (x) -> setPosition(x));
        builder.addDoubleProperty("L Motor Out", () -> leftMotor.get(), null);
        builder.addDoubleProperty("R Motor Out", () -> rightMotor.get(), null);
        builder.addBooleanProperty("PID Enabled", () -> isPIDEnabled(), (x) -> setPIDEnabled(x));
        builder.addDoubleProperty("PID Setpoint", () -> positionSetpoint, (x) -> setPosition(x));
        builder.addBooleanProperty("Left Limit", () -> leftHardLimit(), null);
        builder.addBooleanProperty("Right Limit", () -> rightHardLimit(), null);
        builder.addBooleanProperty("Save Prefs", () -> false, (x) -> { if (x) PrefValue.saveObjectPrefs(this); });
    }

    public void savePositions() {
        Preferences.setDouble(HopperConstants.leftPositionKey,  leftEncoder.getPosition());
        Preferences.setDouble(HopperConstants.rightPositionKey, rightEncoder.getPosition());
    }
}
