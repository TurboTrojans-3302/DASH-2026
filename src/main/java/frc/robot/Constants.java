// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public final class CanIds {
    public static final int DX_SENSOR_CAN_ID = 0;
    // ID 1 through 8 are for swerve drive, defined in swerve config files
    public static final int kShooterMotorCanId = 9;
    public static final int kFeederMotorCanId = 10;
    public static final int kHopperLeftMotorCanId = 11;
    public static final int kHopperRightMotorCanId = 12;
    public static final int kHarvesterMotorCanId = 13;
    public static final int kSecondFeederMotorCanId = 14;
   // public static final int kClimbMotor1 = 14;
   // public static final int kClimbMotor2 = 15;
  }

  public final class RobotConstants {
    public static final double lengthWithBumpers = 0.762; // meters, 30 inches
    public static final double widthWithBumpers = 0.762; // meters,
  }

  public static final class DigitalIO {
    public static final int kHopperLeftContractedLimitSwitchDio = 1;
    public static final int kHopperRightContractedLimitSwitchDio = 2;
  }

  public static final class DriveConstants {
    public static final String maxSpeedKey = "driveMaxSpeed";
    public static final double kMaxSpeedDefault = 4.0; // m/s
    public static final Double kMaxAngularVelocityDefault = 2.0; // radians per second;
    public static final String maxAngularVelocityKey = "driveMaxAngularVelocity";
  }

  public static final class TeleopConstants {
    public static final double kDefaultDriveSpeedScale = 1.0;
    public static final double kDefaultTurnSpeedScale = 1.0;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final int kCopilotControllerPort = 1;
    public static final int kButtonBoardPort = 2;

    public static final class ButtonBox {
      public static final int Left1 = 1;
      public static final int Right1 = 2;
      public static final int Left2 = 3;
      public static final int Right2 = 4;
      public static final int Left3 = 5;
      public static final int Right3 = 6;
      public static final int Left4 = 7;
      public static final int Right4 = 8;
      public static final int Esc = 9;
      public static final int Enter = 10;
      public static final int SafetySwitch = 12;
      public static final int EngineStart = 11;
      public static final int Switch1Up = 13;
      public static final int Switch1Down = 14;
      public static final int Switch2Up = 15;
      public static final int Switch2Down = 16;
      public static final int Switch3Up = 17;
      public static final int Switch3Down = 18;
      public static final int Switch4Up = 19;
      public static final int Switch4Down = 20;
      public static final int LeftKnobCW = 21;
      public static final int LeftKnobCCW = 22;
      public static final int RightKnobCCW = 23;
      public static final int RightKnobCW = 24;
      public static final int LeftKnobPush = 25;
      public static final int RightKnobPush = 26;

      public static final int StickUp = 0;
      public static final int StickUpRight = 45;
      public static final int StickRight = 90;
      public static final int StickDownRight = 135;
      public static final int StickDown = 180;
      public static final int StickDownLeft = 225;
      public static final int StickLeft = 270;
      public static final int StickUpLeft = 315;
      public static final int StickCenter = -1;
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI/2;

    public static final double kDistanceTolerance = 0.050;
    public static final double kHeadingTolerance = 2.0; // degrees
  }

  public static final class FieldConstants {
    public static final Pose2d ZeroZero = new Pose2d(Translation2d.kZero, Rotation2d.kZero);
    public static final Pose2d HubCenterPoint = new Pose2d(4.62, 4.03, Rotation2d.kZero);
    public static final Pose2d HubFrontFaceCenter = new Pose2d(3.6, 4.0346376, Rotation2d.k180deg);
    public static final Pose2d StartCenterTouchingHub = new Pose2d(4.62 - RobotConstants.lengthWithBumpers/2, 4.03, Rotation2d.k180deg);}

  public static final class LimelightConstants {
    public static final String name = "limelight";

    public static final class Offset {

      // measured to the center of the lens

      // inches
      public static final double forward = -0.3;
      public static final double side    = .011;
      public static final double up      = 0.6;

      public static final double roll    = 0.0;
      public static final double pitch   = 0.0;
      public static final double yaw     = 180.0; // Limelight is mounted facing backwards, so we need to rotate the camera pose by 180 degrees around the vertical axis
    }

    public static final class PipelineIdx {
      public static final int AprilTag = 0;
      public static final int NeuralDetector = 1;
      public static final int NeuralClassifer = 2;
    }
  }

  public static final class ShooterConstants {
    public static final double kPdefault = 0.001;
    public static final double kIdefault = 0;
    public static final double kDdefault = 0.0002;
    public static final double kVdefault = 0.00018;
    public static double PIDToleranceDefault = 10.0;
    public static final String kPkey = "shooter_kP";
    public static final String kIkey = "shooter_kI";
    public static final String kDkey = "shooter_kD";
    public static final String kVkey = "shooter_kV";
    public static final String PIDToleranceKey = "shooter_PIDTolerance";
    public static final String kDfilterKey = "shooter_kDfilter";
    public static final double kDfilterDefault = 0.0;
    public static final String kRampRateKey = "shooter_kRampRate";
    public static final double kRampRateDefault = 0.0;
    public static final String kMaxAccelKey = "shooter_kMaxAccel";
    public static final double kMaxAccelDefault = 1500.0;
    public static final String kMaxVelocityKey = "shooter_kMaxVelocity";
    public static final double kMaxVelocityDefault = 3000.0;
    public static final double defaultShootRPM = 1600.0;

    public static final double feederSpeedDefault = 0.3;
    public static final String feederSpeedKey = "feederSpeed";
    public static double manualRPMincrement = 10; // rpm
    public static final String secondFeederSpeedKey = "secondFeederSpeed";
    public static final double velocityFilterTimeConstant = 4 * Robot.kDefaultPeriod;

    public static final double SecondaryFeederSpeedDefault = 0.3;
    public static final int smartCurrentLimitDefault = 60;
    public static final int secondaryCurrentLimitDefault = 60;
    public static final int currentLimitRPMDefault = 5700;

    public static final String smartCurrentLimitKey = "shooter_SmartCurrentLimit";
    public static final String secondaryCurrentLimitKey = "shooter_SecondaryCurrentLimit";
    public static final String currentLimitRPMKey = "shooter_CurrentLimitRPM";
  }

  public static final class HarvesterConstants {
    public static final double kVdefault = .02;
    public static final String kVkey = "harvesterKv";
    public static final double pullInRPMDefault = 400;
    public static final String pullInRPMkey = "harvesterPullInRPM";
    public static final double kPdefault = 0.01;
    public static final String kPkey = "harvesterKp";
    public static final double kTolDefault = 10.0;
    public static final String kTolKey = "harvesterKTol";
  }

  public static final class ClimberConstants {

  }

  public static final class HopperConstants {
    public static final String maxPositionKey = "hopperMaxPosition";
    public static final double maxPositionDefault = 220.0; // encoder units
    public static final String minPositionKey = "hopperMinPosition";
    public static final double minPositionDefault = 3.0; // encoder units
    public static final String leftPositionKey = "hopperLeftPosition";
    public static final double leftPositionDefault = 0.0; // encoder units
    public static final String rightPositionKey = "hopperRightPosition";
    public static final double rightPositionDefault = 0.0; // encoder units
    // public static final String kVkey = "hopperKv";
    // public static final double kVdefault = 0.001;
    public static final String posToleranceKey = "hopperPosTolerance";
    public static final double posToleranceDefault = 1;
    public static final String kPkey = "hopperKp";
    public static final double kPdefault = 0.02;
    public static final String kIkey = "hopperKi";
    public static final double kIdefault = 0.0;
    public static final String kDkey = "hopperKd";
    public static final double kDdefault = 0.0;
    public static final String kGkey = "hopperKg";
    public static final double kGdefault = 0.0;
    public static final String maxVelocityKey = "hopperMaxVelocity";
    public static final double maxVelocityDefault = 80.0; // encoder units per second
    public static final String maxAccelerationKey = "hopperMaxAcceleration";
    public static final double maxAccelerationDefault = 50.0; // encoder units per second squared
    public static final String hardLimitEnableKey = "hopperHardLimitEnable";
  }

  public static final int BLINKIN_LED_PWM_CHANNEL = 0;
}