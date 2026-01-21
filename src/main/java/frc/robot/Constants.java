// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

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

    // Ludwig DriveTrain
    public static final int kFrontRightTurningCanId = 1;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kFrontLeftTurningCanId = 3;
    public static final int kFrontLeftDrivingCanId = 4;
    public static final int kRearLeftTurningCanId = 5;
    public static final int kRearLeftDrivingCanId = 6;
    public static final int kRearRightTurningCanId = 7;
    public static final int kRearRightDrivingCanId = 8;
    // 9
    // 10
    // 11
    // 12
    public static final int kIntakeConveyorCanId = 13;    
    // 14    
    public static final int kElevatorCanId = 15;
    public static final int kGripperMotorCanId = 16;
    public static final int kGripperExtensionMotorCanId = 17;
    public static final int kClimberLeftMotorCanId = 18;
    public static final int kClimberRightMotorCanId = 19;
    public static final int intakeArmMotorID = 20;
    

  }

  public static final class DigitalIO {
        public static final int kElevatorHighLimitSwitchId = 0;
        public static final int kElevatorLowLimitSwitchId = 1;
        public static final int kLowerIntakeLimitSwitchId = 2;
        public static final int kUpperIntakeLimitSwitchId = 3;        
        public static final int kIntakeArmLimitSwitchId = 4;
        public static final int kIntakeArmEncoderDIO = 5;
        public static final int kGripperClosedSwitchId = 6;
        public static final int kGripperFullyRetractedSwitchId = 7;
        public static final int kGripperObjectDetectedSwitchId = 8;
        
      }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final int kCopilotControllerPort = 1;
    public static final int kReefControllerPort = 3;
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
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 50;
    public static final double kMaxAccelerationMetersPerSecondSquared = 50;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    public static final double kDistanceTolerance = 0.050;
    public static final double kHeadingTolerance = 2.0; // degrees
  }

  public static final class FieldConstants {
    public static final Pose2d ZeroZero = new Pose2d(0.0, 0.0, new Rotation2d());
    // for the station with april tag 13 these values both stay positive
    public static final double poseOffsetStationRightX = 0.5;
    public static final double poseOffsetStationRightY = 0.5;
    //relative in the y-direction
    public static final double yOffsetReefPoleLeft = 0.3;
    public static final double yOffsetReefPoleRight = -0.3;
  }

  public static final class LimelightConstants {
    public static final String name = "limelight";

    public static final class Offset {

      //measured to the center of the lens

      //inches
      public static final double forward = 4; 
      public static final double side = -7.5; //right is negative
      public static final double up = 37.25; 

      public static final double roll = 0.0;
      public static final double pitch = 0.0;
      public static final double yaw = 0.0;
    }

    public static final class PipelineIdx {
      public static final int AprilTag = 0;
      public static final int NeuralDetector = 1;
      public static final int NeuralClassifer = 2;
    }
  }

  public static final class ElevatorConstants {
    // position in motor rotations for different levels of the reef, all random
    // numbers that need to be tested of course
    public static final double kLimitLow = 0;
    public static final double kSoftLimitLow = 10;
    public static final double kPickupLevel = 100;
    public static final double kLevel2 = 300;
    public static final double kLevel3 = 400;
    public static final double kLevel4 = 500;
    public static final double kSoftLimitHigh = 550;
    public static final double kLimitHigh = 563.9;
    // honestly the processor probably wouldn't require the elevator to move at all
    // from the start position, but if it is already up it could be useful to have a
    // preset position
    public static final double kProcessor = 15;

    public static final double kElevatorAutoSpeedToLevel = 1.0;
    public static final double kElevatorMaxSpeed = 100000;
    public static final double kElevatorMaxAccel = 100000;
    public static final double kManualRate = 1.0;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kLoadPosition = 0;
    public static final double kTolerance = 3; //position tolerance
  }

  public static final class IntakeConstants {
    public static final double intakeSpeedMax = 1.0;
    public static final double intakeSpeedAuto = 0.4;
    public static final double kI = 0.00;
    public static final double kP = 0.03;
    public static final double kD = 0.001;
    public static final double intakeSpeedMin = -1.0;
    // public static final double armAngleOffsetLeft = -18.1;
    // public static final double armAngleOffsetRight = 113.4;
    public static final double armAngleOffset = 144;
    public static final double MaxArmAngle = 115.0;
    public static final double MinArmAngle = -6.0;
    public static final double upSpeed = -0.4;
    public static final double downSpeed = 1.0;
    public static final double kS = 0.0;
    public static final double kG = 0.15;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double kMaxVelocity = 720;
    public static final double kMaxAcceleration = 200;
    public static final double kFloorPosition = MinArmAngle;
    public static final double kElevatorPosition = MaxArmAngle;
    public static final double kTroughPosition = 73;
    public static final double upperLoadSpeed = 0;
    public static final double ejectTime = 1.0;
    public static final double kPositionTolerance = 2.0;
    public static final double kPositionIncrement = 1.0;
  }

  public static final class ClimberConstants {
    public static final double kP = 1.0;
    public static final double kI = 0;
    public static final double kD = 0.5;
    public static final double kMaxVelocity = 100000; //RPM
    public static final double kMaxAcceleration = 100000; //RPM/s
    public static double kLowerLimit = -1.5;
    public static double kLockedPosition = 7;
    public static double kUpperLimitRight = 16;
    public static double kUpperLimitLeft = 22;
    public static double increment = 1.0;
  }

  public static final class GripperConstants {
    public static final double gripperExtendedPosition = 65.0;
    public static final double gripperRetractedPosition = 0.0;
    // only one speed necessary
    public static final double gripperExtensionSpeed = 0.5;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double openPosition = 100.0;
    public static final double closedPosition = 0.0;
  }

  public static final int BLINKIN_LED_PWM_CHANNEL = 0;
}