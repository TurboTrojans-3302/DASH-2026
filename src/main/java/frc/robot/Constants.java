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
    
    // 14    
    
    

  }

  public static final class DigitalIO {
        
        
      }
  public static final class DriveConstants {
    public static final double kMaxSpeed = 4.0; // m/s
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

  public static final class ShooterConstants {
    public static final double kPdefault = 0;
    public static final double kIdefault = 0;
    public static final double kDdefault = 0;
    public static final double kVdefault = 0;
    public static double PIDToleranceDefault = 0.0;
    public static final String kPkey = "shooter_kP";
    public static final String kIkey = "shooter_kI";
    public static final String kDkey = "shooter_kD";
    public static final String kVkey = "shooter_kV";
    public static final String PIDToleranceKey = "shooter_PIDTolerance";
    public static final double maxRPM = 4000.0;

    public static final double feederSpeedDefault = 0;
    public static final String feederSpeedKey = "feederSpeed";
    public static double manualSpeedIncrement = 20; //rpm

  }

  public static final class IntakeConstants {
    
  }

  public static final class ClimberConstants {
   
  }

 

  public static final int BLINKIN_LED_PWM_CHANNEL = 0;
}