// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ElevatorUpTime;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(DriveSubsystem subsystem) {
    //return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
    return null;
  }

  public static Command driveForwardAuto(DriveSubsystem subsystem) {
    return new DriveForwardAuto(subsystem, 3, 0.25);
  }

  public static Command ScoreL1Auto(DriveSubsystem subsystem, CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem) {
    return Commands.sequence(new DriveForwardAuto(subsystem, .625, 0.25), new DriveForwardAuto(subsystem, 3, 0.1), new ElevatorUpTime(elevatorSubsystem, 1), new ScoreL1(coralSubsystem));
  }


  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
