// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.utils.AprilUtils.Angle;
import frc.utils.AprilUtils.AngularUnit;

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
  public static class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.15;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 2.2; // radians per second
    public static final double kMagnitudeSlewRate = 2.4; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 12;
    public static final int kRearLeftDrivingCanId = 11;
    public static final int kFrontRightDrivingCanId = 13;
    public static final int kRearRightDrivingCanId = 10;

    public static final int kFrontLeftTurningCanId = 22;
    public static final int kRearLeftTurningCanId = 21;
    public static final int kFrontRightTurningCanId = 23;
    public static final int kRearRightTurningCanId = 20;

    public static final boolean kGyroReversed = true;

    public static final double kXDiffCoordP = 0.04;
    public static final double kXDiffCoordI = 0;
    public static final double kXDiffCoordD = 0;

    public static final double kYDiffCoordP = 0.04;
    public static final double kYDiffCoordI = 0;
    public static final double kYDiffCoordD = 0;

    public static final double kHeadingP = 0.6;
    public static final double kHeadingI = 0;
    public static final double kHeadingD = 0;

    public static final double rotSpeedCoefficient = 0.8;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 12;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI); // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.01;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 0.8;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0.0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class CoralConstants {
    public static final int kCoralMotorCanID = 61;
    public static final double kCoralSpeed = 1;
    public static final int kLimitSwitchChannel = 5;
  }

  public static final class ElevatorConstants {
    public static final int kBackLiftCanID = 41;
    public static final int kFrontLiftCanID = 42;
    public static final int kElevatorPulleyChannelID = 0; // DIO port
    public static final int kElevatorMotorCurrentLimit = 50; // amps
    public static final double kElevatorMotorRampRate = 1.0; // in seconds
    public static final double kElevatorSpeed = .75; // percent
    public static final double kHighStopEncoder = 5;
    public static final double kLowStopEncoder = -1;
    public static final double kHighStopDistanceSensor = 100;
    public static final double kLowStopDistanceSensor = 2;
    public static final double kElevatorP = 0.113;
    public static final double kElevatorI = 0.0035;
    public static final double kElevatorD = 0.0002;
  }

  public static final class AlgaeConstants {
    public static final int kRoller1MotorCanId = 51;
    public static final int kRoller2MotorCanId = 31;
    public static final int kRollerMotorCurrentLimit = 20;
    public static final double kRollerMotorRampRate = 1.0; // in seconds
    public static final double kRollerSpeed = 1; // percent
  }

  public static final class ArmConstants {
    public static final int kArmMotorCanId = 62;
    public static final int kArmMotorCurrentLimit = 50;
    public static final double kArmMotorRampRate = 0.125; // in seconds
    public static final double kArmSpeed = 0.2; // percent
    public static final double kArmP = 0.02;
    public static final double kArmI = 0;
    public static final double kArmD = 0;
    public static final double kArmClockwiseStop = .3 + 1; // Adding 1 for bounds
    public static final double kArmCounterClockwiseStop = 0.706;  // bounds are from 0.548 -> 0.11; it crosses over the 0 while within out bounds
    public static final double kMiddleBadBounds = 0.5;
    public static final double kLowerLevelsCoralScore = 0.987;
    public static final double kHome = 0.87;
    public static final double kIntakeCoralPosition = 0.60;
    public static final double kIntakeAlgaePosition = 0.99;
    public static final double kHighLevelCoralScore = 0.116 + 1; // Adding 1 for bounds
    public static final double kHighAlgaeScore = 0.96; // ?
  }

  public static final class ElevatorPositionConstants {
    public static final double kHome = 6;//?
    public static final double kLevel1 = 7;//?
    public static final double kLevel2 = 10;
    public static final double kLevel3 = 39;
    public static final double kLevel4 = 100;//?
    public static final double kIntakeCoralPosition = 36;
    public static final double kIntakeAlgae23Position = 12; // ?
    public static final double kIntakeAlgae34Position = 42; // ?
    public static final double kScoreHighAlgaePosition = 95; // ?
  }

  public static final class LimelightConstants {
    public static final String leftLimelight = "limelight-allison";
    public static final double leftXOffset = 0;
    public static final double leftYOffset = 0;
    public static final double leftHeight = 0;
    public static final Angle leftHorizontalAngle = new Angle(0, AngularUnit.DEGREES);
    public static final Angle leftVerticalAngle = new Angle(0, AngularUnit.DEGREES);
    public static final String rightLimelight = "limelight";
    public static final double rightXOffset = 0;
    public static final double rightYOffset = 0;
    public static final double rightHeight = 0;
    public static final Angle rightHorizontalAngle = new Angle(0, AngularUnit.DEGREES);
    public static final Angle rightVerticalAngle = new Angle(0, AngularUnit.DEGREES);
  }

  public static final class AutoAimConstants {
    public static final int kScoreLeftInput = 90;
    public static final int kScoreRightInput = 270;
    public static final double autoAimXOffset = 19; // ?
    public static final double autoAimYOffset = 0; // ?
    public static final double kDriveOffsetY = 0; // ?
  }
}
