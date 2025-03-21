// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;


public class ElevatorUpTime extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem elevatorSubsystem;
  // Distance to drive forward
  // If the robot reached the distance
  
  private double time;

  private double duration;

  /**
   * Creates the drive forward auto
   * Drives forward a set distance
   * @param distance double The distance to drive
   * @param slow boolean If the robot should drive slower
   */
  public ElevatorUpTime(ElevatorSubsystem subsystem, double duration) {
    elevatorSubsystem = subsystem;
    this.duration = duration;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drives forward and returns if it reached the destination
    elevatorSubsystem.elevatorMove(ElevatorConstants.kElevatorSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // When the robot is at the destination, set the speed to 0
    elevatorSubsystem.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp()-time >= duration)
      return true;
    return false;
  }
}

