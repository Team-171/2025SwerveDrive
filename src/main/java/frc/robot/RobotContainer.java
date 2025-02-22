// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.MoveCoral;
import frc.robot.commands.MoveRoller;
import frc.robot.commands.MoveArm;
import frc.robot.commands.autos.Autos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    /* private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();
    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem(); */
    private SendableChooser<Command> autoChooser;

    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
//     XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        /* autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Drive Forward", Autos.driveForwardAuto(m_driveSubsystem)); */
        // autoChooser.addOption("ScoreL1", Autos.ScoreL1Auto(m_driveSubsystem, m_coralSubsystem, m_elevatorSubsystem));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(autoChooser);


        SmartDashboard.putData(m_driveSubsystem);

        // Configure the trigger bindings
        configureBindings();

        // Configure default commands
        m_driveSubsystem.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_driveSubsystem.drive(
                                -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                        OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                        OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getRightX(),
                                        OIConstants.kDriveDeadband),
                                m_driverController.getStartButton()),
                        m_driveSubsystem));

       /*/ m_elevatorSubsystem.setDefaultCommand(
                new RunCommand(
                        () -> m_elevatorSubsystem.holdCurrentPosition(),
                        m_elevatorSubsystem)); */

    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {

        // set lock formation of the drive
        new JoystickButton(m_driverController, XboxController.Button.kX.value)
                .whileTrue(new RunCommand(
                        () -> m_driveSubsystem.setX(),
                        m_driveSubsystem));

        // zero the heading
        new JoystickButton(m_driverController, XboxController.Button.kBack.value) // select button
                .onTrue(new RunCommand(() -> m_driveSubsystem.zeroHeading()));

        /* new JoystickButton(m_driverController, XboxController.Button.kA.value) // move coral in
                .whileTrue(new MoveCoral(m_coralSubsystem, CoralConstants.kCoralSpeed));

        new JoystickButton(m_driverController, XboxController.Button.kB.value) // move coral out
                .whileTrue(new MoveCoral(m_coralSubsystem, -CoralConstants.kCoralSpeed));

        new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value) // move the elevator down manually
                .whileTrue(new ElevatorCommand(m_elevatorSubsystem, -ElevatorConstants.kElevatorSpeed));

        new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value) // move the elevator up manually
                .whileTrue(new ElevatorCommand(m_elevatorSubsystem, ElevatorConstants.kElevatorSpeed)); */
   
        /* new JoystickButton(m_operatorController, XboxController.Button.kX.value)        // operator move arm down
                .whileTrue(new MoveArm(m_algaeSubsystem, AlgaeConstants.kArmSpeed));

        new JoystickButton(m_operatorController, XboxController.Button.kY.value)        // operator move arm up
                .whileTrue(new MoveArm(m_algaeSubsystem, -AlgaeConstants.kArmSpeed));

        new JoystickButton(m_operatorController, XboxController.Button.kA.value)        // operator move roller
                .whileTrue(new MoveRoller(m_algaeSubsystem, AlgaeConstants.kRollerSpeed));

        new JoystickButton(m_operatorController, XboxController.Button.kB.value)
                .whileTrue(new MoveRoller(m_algaeSubsystem, -AlgaeConstants.kRollerSpeed));     // operator move roller */
        }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
