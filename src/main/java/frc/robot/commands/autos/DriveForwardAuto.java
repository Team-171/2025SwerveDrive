// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Line up the robots bumper that is closest to the middle with the april tag on the blue alliance barge
 */
public class DriveForwardAuto extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem driveSubsystem;
  // Distance to drive forward
  // If the robot reached the distance
  
  private double time;

  private double duration;

  private double speed;

  /**
   * Creates the drive forward auto
   * Drives forward a set distance
   * @param distance double The distance to drive
   * @param slow boolean If the robot should drive slower
   */
  public DriveForwardAuto(DriveSubsystem subsystem, double duration, double speed) {
    driveSubsystem = subsystem;
    this.duration = duration;
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = Timer.getFPGATimestamp();
    // Pose2d startingPose = new Pose2d(310.5, 256.64, new Rotation2d(180));
    // driveSubsystem.resetOdometry(startingPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drives forward and returns if it reached the destination
    driveSubsystem.drive(speed, 0, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // When the robot is at the destination, set the speed to 0
    driveSubsystem.drive(0,0,0,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp()-time >= duration)
      return true;
    return false;
  }
}

