package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;
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
    private double softMax = HopperConstants.maxPositionDefault;
    private double softMin = HopperConstants.minPositionDefault;
    //private double kV = HopperConstants.kVdefault;
    private double kP = HopperConstants.kPdefault;
    private double kI = HopperConstants.kIdefault;
    private double kD = HopperConstants.kDdefault;
    private double posTolerance = HopperConstants.posToleranceDefault;
    private double kG = HopperConstants.kGdefault;
    private double maxVelocity     = HopperConstants.maxVelocityDefault;
    private double maxAcceleration = HopperConstants.maxAccelerationDefault;
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
        loadPreferences();

        leftMotor = new SparkMax(leftMotorID, MotorType.kBrushless);
        rightMotor = new SparkMax(rightMotorID, MotorType.kBrushless);

        configureSparkMaxes();

        leftMotor.set(0);
        leftEncoder = leftMotor.getEncoder();
        leftEncoder.setPosition(0); // reset encoder position on startup

        rightMotor.set(0);
        rightEncoder = rightMotor.getEncoder();
        rightEncoder.setPosition(0); // reset encoder position on startup

    }

    private void configureSparkMaxes() {
        SparkMaxConfig leftSparkConfig = new SparkMaxConfig();
        leftSparkConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);

        SparkMaxConfig rightSparkConfig = new SparkMaxConfig().apply(leftSparkConfig);
        rightSparkConfig.inverted(true); // invert right motor 

        leftMotor.configure(leftSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        rightMotor.configure(rightSparkConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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

    public void move(double speed) {
        if (isPIDEnabled()) {
            // Do nothing
        } else {
            double motorSpeed = MathUtil.clamp(speed, -1.0, 1.0);
            setMotorPctOutput(motorSpeed, motorSpeed);
        }
    }

    public void setPosition(double pos) {
        if(isPIDEnabled()) {
            positionSetpoint = MathUtil.clamp(pos, softMin, softMax);
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
        return leftEncoder.getPosition() >= softMax;
    }

    public boolean atSoftMaxR() {
        return rightEncoder.getPosition() >= softMax;
    }

    public boolean atMaxPosition() {
        return atSoftMaxL() && atSoftMaxR();
    }

    public boolean atSoftMinL() {
        return leftEncoder.getPosition() <= softMin;
    }

    public boolean atSoftMinR() {
        return rightEncoder.getPosition() <= softMin;
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
        Command cmd = setPositionCommand(() -> softMax);
        cmd.setName("expandCommand");
        return cmd;
    }

    public Command retractCommand() {
        Command cmd = setPositionCommand(() -> softMin);
        cmd.setName("retractCommand");
        return cmd;
    }

    public Command manualMoveCommand(DoubleSupplier speedSupplier) {
        Command cmd = new FunctionalCommand(
            () -> {}, // no init
            () -> move(speedSupplier.getAsDouble()), // call move() with supplier value
            (interrupted) -> { stop(); }, // stop on end or interrupt
            () -> false, // never end on its own
            this);
        cmd.setName("manualMoveCommand");
        return cmd;
    }

    /** Apply current kP/kI/kD/kG/tolerance/constraints to both PID controllers and feedforward. */
    private void applyPIDGains() {
        leftPID.setPID(kP, kI, kD);
        rightPID.setPID(kP, kI, kD);
        leftPID.setTolerance(posTolerance);
        rightPID.setTolerance(posTolerance);
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);
        leftPID.setConstraints(constraints);
        rightPID.setConstraints(constraints);
        feedforward = new ElevatorFeedforward(0, kG, 0);
    }

    private void loadPreferences() {
        if (Preferences.containsKey(HopperConstants.maxPositionKey)) {
            System.out.println("Loading Hopper  values from preferences");
            softMax = Preferences.getDouble(HopperConstants.maxPositionKey,
                    HopperConstants.maxPositionDefault);
            softMin = Preferences.getDouble(HopperConstants.minPositionKey,
                    HopperConstants.minPositionDefault);
            //kV = Preferences.getDouble(HopperConstants.kVkey, HopperConstants.kVdefault);
            kP = Preferences.getDouble(HopperConstants.kPkey, HopperConstants.kPdefault);
            kI = Preferences.getDouble(HopperConstants.kIkey, HopperConstants.kIdefault);
            kD = Preferences.getDouble(HopperConstants.kDkey, HopperConstants.kDdefault);
            kG = Preferences.getDouble(HopperConstants.kGkey, HopperConstants.kGdefault);
            posTolerance = Preferences.getDouble(HopperConstants.posToleranceKey, HopperConstants.posToleranceDefault);
            maxVelocity     = Preferences.getDouble(HopperConstants.maxVelocityKey,     HopperConstants.maxVelocityDefault);
            maxAcceleration = Preferences.getDouble(HopperConstants.maxAccelerationKey, HopperConstants.maxAccelerationDefault);
            applyPIDGains();
        } else {
            System.out.println("No hopper prefs found. Using default values");
        }
    }

    private void savePreferences() {
        System.out.println("Saving hopper values to preferences");
        Preferences.setDouble(HopperConstants.maxPositionKey, softMax);
        Preferences.setDouble(HopperConstants.minPositionKey, softMin);
        //Preferences.setDouble(HopperConstants.kVkey, kV);
        Preferences.setDouble(HopperConstants.kPkey, kP);
        Preferences.setDouble(HopperConstants.kIkey, kI);
        Preferences.setDouble(HopperConstants.kDkey, kD);
        Preferences.setDouble(HopperConstants.kGkey, kG);
        Preferences.setDouble(HopperConstants.posToleranceKey, posTolerance);
        Preferences.setDouble(HopperConstants.maxVelocityKey,     maxVelocity);
        Preferences.setDouble(HopperConstants.maxAccelerationKey, maxAcceleration);
    }

    public boolean leftHardLimit() {
        return leftContractedLimitSwitch.get();
    }

    public boolean rightHardLimit() {
        return rightContractedLimitSwitch.get();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Max Position", () -> softMax, (x) -> { softMax = x; });
        builder.addDoubleProperty("Min Position", () -> softMin, (x) -> { softMin = x; });
        builder.addDoubleProperty("Left Position", () -> leftEncoder.getPosition(), null);
        builder.addDoubleProperty("Right Position", () -> rightEncoder.getPosition(), null);
        builder.addDoubleProperty("Position Setpoint", () -> positionSetpoint, (x) -> setPosition(x));
        builder.addDoubleProperty("L Motor Out", () -> leftMotor.get(), null);
        builder.addDoubleProperty("R Motor Out", () -> rightMotor.get(), null);
        //builder.addDoubleProperty("kV", () -> kV, (x) -> { kV = x; });
        builder.addDoubleProperty("kP", () -> kP, (x) -> { kP = x; applyPIDGains(); });
        builder.addDoubleProperty("kI", () -> kI, (x) -> { kI = x; applyPIDGains(); });
        builder.addDoubleProperty("kD", () -> kD, (x) -> { kD = x; applyPIDGains(); });
        builder.addDoubleProperty("kG", () -> kG, (x) -> { kG = x; applyPIDGains(); });
        builder.addDoubleProperty("Max Velocity",     () -> maxVelocity,     (x) -> { maxVelocity     = x; applyPIDGains(); });
        builder.addDoubleProperty("Max Acceleration", () -> maxAcceleration, (x) -> { maxAcceleration = x; applyPIDGains(); });
        builder.addBooleanProperty("PID Enabled", () -> isPIDEnabled(), (x) -> setPIDEnabled(x));
        builder.addDoubleProperty("PID Setpoint", () -> positionSetpoint, (x) -> setPosition(x));
        builder.addBooleanProperty("Left Limit", () -> leftHardLimit(), null);
        builder.addBooleanProperty("Right Limit", () -> rightHardLimit(), null);
        builder.addBooleanProperty("Save Prefs", () -> false, (x) -> { if (x) savePreferences(); });
    }
}
