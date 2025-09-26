// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.helperObjects.StateEnum;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ElevatorPositionConstants;
import frc.robot.commands.ArmPresetCommand;
import frc.robot.commands.ElevatorPresetCommand;
import frc.robot.commands.ElevatorUpTime;
import frc.robot.commands.GoToStateCommand;
import frc.robot.commands.MoveAlgae;
import frc.robot.commands.OutputCoral;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public final class Autos {
    /** Example static factory for an autonomous command. */
    public static Command exampleAuto(DriveSubsystem subsystem) {
        // return Commands.sequence(subsystem.exampleMethodCommand(), new
        // ExampleCommand(subsystem));
        return null;
    }

    public static Command driveForwardAuto(DriveSubsystem subsystem) {
        return new DriveForwardAuto(subsystem, 3, 0.25);
    }

    public static Command ScoreL1Auto(DriveSubsystem subsystem, CoralSubsystem coralSubsystem,
            ElevatorSubsystem elevatorSubsystem) {
        return Commands.sequence(new DriveForwardAuto(subsystem, .625, 0.25), new DriveForwardAuto(subsystem, 3, 0.1),
                new ElevatorUpTime(elevatorSubsystem, 1), new ScoreL1(coralSubsystem));
    }

    public static Command ScoreL4Auto(DriveSubsystem subsystem, CoralSubsystem coralSubsystem,
            ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        Command command = Commands.sequence(new DriveForwardAuto(subsystem, 0.625, 0.25),
                new DriveForwardAuto(subsystem, 2, 0.1));
        command = Commands.sequence(command,
                new GoToStateCommand(elevatorSubsystem, armSubsystem, StateEnum.CORAL_LEVEL_4));
        command = Commands.sequence(command,
                Commands.race(new HoldPosition(armSubsystem, elevatorSubsystem), new WaitCommand(3)));
        command = Commands.sequence(command,
                Commands.race(new HoldPosition(armSubsystem, elevatorSubsystem),
                        new OutputCoral(coralSubsystem, CoralConstants.kCoralSpeed), new WaitCommand(1)));

        return command;
    }

    public static Command ScoreL4AutoWithAlgae(DriveSubsystem subsystem, CoralSubsystem coralSubsystem,
            ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, AlgaeSubsystem algaeSubsystem) {
        Command command = Commands.sequence(new DriveForwardAuto(subsystem, 1.05, 0.15),
                new DriveForwardAuto(subsystem, 2, 0.1));
        command = Commands.sequence(command,
                new GoToStateCommand(elevatorSubsystem, armSubsystem, StateEnum.CORAL_LEVEL_4));
        command = Commands.sequence(command,
                Commands.race(new HoldPosition(armSubsystem, elevatorSubsystem), new WaitCommand(3)));
        command = Commands.sequence(command,
                Commands.race(new HoldPosition(armSubsystem, elevatorSubsystem),
                        new OutputCoral(coralSubsystem, CoralConstants.kCoralSpeed), new WaitCommand(0.65)));
        command = Commands.sequence(command,
                new GoToStateCommand(elevatorSubsystem, armSubsystem, StateEnum.INTAKE_ALGAE_23));
        command = Commands.sequence(command,
                Commands.race(new HoldPosition(armSubsystem, elevatorSubsystem), new WaitCommand(2)));
        command = Commands.sequence(command,
                Commands.race(new HoldPosition(armSubsystem, elevatorSubsystem),
                        new DriveSidewaysAuto(subsystem, .5, 0.1)));
        command = Commands.sequence(command,
                Commands.race(new HoldPosition(armSubsystem, elevatorSubsystem),
                        new MoveAlgae(algaeSubsystem, AlgaeConstants.kRollerSpeed), new WaitCommand(2)));
        return command;
    }

    public static Command LeftRightScoreL1(DriveSubsystem driveSubsystem, CoralSubsystem coralSubsystem,
            ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        Command command = Commands
                .race(Commands.sequence(
                        new GoToStateCommand(elevatorSubsystem, armSubsystem, StateEnum.INTAKE_CORAL),
                        new HoldPosition(armSubsystem, elevatorSubsystem)),
                        new DriveForwardAuto(driveSubsystem, 2.5, -0.25));
        command = Commands.sequence(command, Commands.race(
                new OutputCoral(coralSubsystem, 0.5),
                new HoldPosition(armSubsystem, elevatorSubsystem),
                new WaitCommand(2)));
        return command;
    }

    public static Command LeftRightScoreL1Take2(DriveSubsystem driveSubsystem, CoralSubsystem coralSubsystem,
            ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        Command command = Commands.race(
                Commands.sequence(
                        new GoToStateCommand(elevatorSubsystem, armSubsystem, StateEnum.INTAKE_CORAL),
                        new HoldPosition(armSubsystem, elevatorSubsystem)),
                new DriveForwardAuto(driveSubsystem, 1, -0.25));
        command = Commands.sequence(command, Commands.race(new RotateAuto(driveSubsystem, .25, .25),
                new HoldPosition(armSubsystem, elevatorSubsystem)));
        command = Commands.sequence(command, Commands.race(new DriveForwardAuto(driveSubsystem, 1, -0.25),
                new HoldPosition(armSubsystem, elevatorSubsystem)));
        command = Commands.sequence(command, Commands.race(
                new OutputCoral(coralSubsystem, 0.5),
                new HoldPosition(armSubsystem, elevatorSubsystem),
                new WaitCommand(2)));
        return command;
    }

    public static SendableChooser<Command> buildChooser(DriveSubsystem driveSubsystem, CoralSubsystem coralSubsystem,
            ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, AlgaeSubsystem algaeSubsystem) {
        SendableChooser<Command> autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("DriveForwardNotPathPlanner",
                new DriveForwardAuto(driveSubsystem, 1, 0.5));
        autoChooser.addOption("L4 Auto", Autos.ScoreL4Auto(driveSubsystem, coralSubsystem,
                elevatorSubsystem, armSubsystem));
        autoChooser.addOption("L4 Auto With Algae", Autos.ScoreL4AutoWithAlgae(driveSubsystem,
                coralSubsystem, elevatorSubsystem, armSubsystem, algaeSubsystem));
        autoChooser.addOption("Side Auto L1", Autos.LeftRightScoreL1(driveSubsystem, coralSubsystem,
                elevatorSubsystem, armSubsystem));
        autoChooser.addOption("Side Auto L2 Take 2", Autos.LeftRightScoreL1Take2(driveSubsystem, coralSubsystem, elevatorSubsystem, armSubsystem));
        return autoChooser;
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
