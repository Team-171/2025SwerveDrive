// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.hal.SimLong;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.helperObjects.DriveSideEnum;
import frc.helperObjects.LevelEnum;
import frc.helperObjects.StateEnum;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorPositionConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmPresetCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorPresetCommand;
import frc.robot.commands.ElevatorToPreset;
import frc.robot.commands.GoToStateCommand;
import frc.robot.commands.OutputCoral;
import frc.robot.commands.SetScoreSideCoral;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.MoveAlgae;
import frc.robot.commands.MoveArm;
import frc.robot.commands.SetSelectedPosition;
import frc.robot.commands.ToggleAlgaeIntakeCommand;
import frc.robot.commands.autos.DriveForwardAuto;
import frc.robot.subsystems.LimitSwitchSubsystem;
import frc.robot.subsystems.ScorePositionSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LedSubsystem;

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

        private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();

        private final ArmSubsystem m_armSubsystem = new ArmSubsystem();

        private final ScorePositionSubsystem m_ScorePositionSubsystem = new ScorePositionSubsystem();
        private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

        private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();

        /*
         * private final LimitSwitchSubsystem m_switchSubsystem = new
         * LimitSwitchSubsystem();
         * private final LedSubsystem m_ledSubsystem = new LedSubsystem();
         */

        private SendableChooser<Command> autoChooser;

        XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
        XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                autoChooser = new SendableChooser<>();
                // autoChooser.setDefaultOption("Drive Forward",
                //                 Autos.driveForwardAuto(m_driveSubsystem));

                // autoChooser.addOption("ScoreL1", Autos.ScoreL1Auto(m_driveSubsystem,
                //                 m_coralSubsystem, m_elevatorSubsystem));

                /*
                 * NamedCommands.registerCommand("L2 Elevator", new
                 * ElevatorPresetCommand(m_elevatorSubsystem,
                 * ElevatorPositionConstants.kLevel2));
                 * NamedCommands.registerCommand("L2 Arm", new ArmPresetCommand(m_armSubsystem,
                 * ArmConstants.kLowerLevelsCoralScore));
                 * NamedCommands.registerCommand("Default Elevator", new RunCommand(() ->
                 * m_elevatorSubsystem.holdCurrentPosition(), m_elevatorSubsystem));
                 * NamedCommands.registerCommand("Default Arm", new RunCommand(() ->
                 * m_armSubsystem.holdCurrentPosition(), m_armSubsystem));
                 */

                // autoChooser = AutoBuilder.buildAutoChooser();
                autoChooser.addOption("DriveForwardNotPathPlanner", new DriveForwardAuto(m_driveSubsystem, 1, 0.5));
                SmartDashboard.putData(autoChooser);

                // Configure the trigger bindings
                configureBindings();

                // Configure default commands
                // ---------------- APPLY SLEW RATE LIMITER TO CONTROLLER INPUTS
                // --------------------
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
                                                                false),
                                                m_driveSubsystem));

                /*
                 * m_switchSubsystem.setDefaultCommand(
                 * new RunCommand(() -> m_switchSubsystem.controllerRumble(m_driverController,
                 * limitSwitch),
                 * m_switchSubsystem));
                 * 
                 * m_ledSubsystem.setDefaultCommand(
                 * new RunCommand(() -> m_ledSubsystem.limitSwitchLed(limitSwitch),
                 * m_ledSubsystem));
                 */

                m_elevatorSubsystem.setDefaultCommand(
                                new RunCommand(
                                                () -> m_elevatorSubsystem.holdCurrentPosition(),
                                                m_elevatorSubsystem));

                // Manual Move of arm
                m_armSubsystem.setDefaultCommand(
                                new RunCommand(
                                                () -> m_armSubsystem.moveArm((m_driverController.getRightTriggerAxis()
                                                                - m_driverController.getLeftTriggerAxis())
                                                                * ArmConstants.kArmSpeed),
                                                m_armSubsystem));

                
                // m_armSubsystem.setDefaultCommand(
                //         new RunCommand(
                //                 () -> m_armSubsystem.holdCurrentPosition(),
                //                 m_armSubsystem));
                

                /*
                 * m_ScorePositionSubsystem.setDefaultCommand(new RunCommand(
                 * () ->
                 * m_ScorePositionSubsystem.setScoreLeftOrRight(m_operatorController.getPOV()),
                 * m_ScorePositionSubsystem));
                 */
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

                // set lock formation of the drive, not used
                // new JoystickButton(m_driverController, XboxController.Button.kX.value)
                // .whileTrue(new RunCommand(
                // () -> m_driveSubsystem.setX(),
                // m_driveSubsystem));

                // zero the heading
                new JoystickButton(m_driverController, XboxController.Button.kBack.value) // select button
                                .onTrue(new RunCommand(() -> m_driveSubsystem.zeroHeading()));

                // move coral out
                new JoystickButton(m_driverController, XboxController.Button.kB.value)
                                .whileTrue(new OutputCoral(m_coralSubsystem, CoralConstants.kCoralSpeed));

                // move coral in
                new JoystickButton(m_driverController, XboxController.Button.kA.value)
                                .whileTrue(new IntakeCoral(m_coralSubsystem, CoralConstants.kCoralSpeed));

                // move algae out
                new JoystickButton(m_driverController, XboxController.Button.kY.value)
                                .whileTrue(new MoveAlgae(m_algaeSubsystem, -AlgaeConstants.kRollerSpeed));

                // move algae in
                new JoystickButton(m_driverController, XboxController.Button.kX.value)
                                .whileTrue(new MoveAlgae(m_algaeSubsystem, AlgaeConstants.kRollerSpeed));

                // ---------------------- Multicomponent Movement ---------------------
                // go to score the coral (moves elevator and arm to position)
                // new Trigger(() -> { return m_driverController.getRightTriggerAxis() > 0.05;
                // })
                // .onTrue(new GoToStateCommand(m_elevatorSubsystem, m_armSubsystem,
                // m_ScorePositionSubsystem.getLevelState()));

                // // go to intake coral (moves elevator and arm to position)
                // new JoystickButton(m_driverController,
                // XboxController.Button.kRightBumper.value)
                // .onTrue(new GoToStateCommand(m_elevatorSubsystem, m_armSubsystem,
                // StateEnum.INTAKE_CORAL));

                // go to score algae high (moves elevator and arm to position)
                // new Trigger(() -> { return m_driverController.getLeftTriggerAxis() > 0.05; })
                // .onTrue(new GoToStateCommand(m_elevatorSubsystem, m_armSubsystem,
                // StateEnum.SCORE_ALGAE_BARGE));

                // go to intake algae (moves elevator and arm to position)
                // new JoystickButton(m_driverController,
                // XboxController.Button.kLeftBumper.value)
                // .onTrue(new ToggleAlgaeIntakeCommand(m_elevatorSubsystem, m_armSubsystem))
                // .onTrue(new ArmPresetCommand(m_armSubsystem,
                // ArmConstants.kIntakeAlgaePosition));

                // go to home position (moves elevator and arm to home position)
                // new JoystickButton(m_driverController, XboxController.Button.kStart.value)
                // .onTrue(new GoToStateCommand(m_elevatorSubsystem, m_armSubsystem,
                // StateEnum.HOME));

                // ------------------------ Preset Testing ----------------------
                // Elevator preset testing use
                // set the elevator hold position with the position from the score position
                // subsystem
                new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
                                .onTrue(new ElevatorToPreset(m_elevatorSubsystem, m_ScorePositionSubsystem));

                // Arm preset testing use
                // new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
                //                 .onTrue(new ArmPresetCommand(m_armSubsystem, ArmConstants.kLowerLevelsCoralScore));

                // ------------------------ Operator Controller ----------------------------
                // set hold position of elevator
                new JoystickButton(m_operatorController, XboxController.Button.kA.value)
                                .onTrue(new SetSelectedPosition(m_ScorePositionSubsystem, StateEnum.CORAL_LEVEL_1));
                new JoystickButton(m_operatorController, XboxController.Button.kB.value)
                                .onTrue(new SetSelectedPosition(m_ScorePositionSubsystem, StateEnum.CORAL_LEVEL_2));
                new JoystickButton(m_operatorController, XboxController.Button.kX.value)
                                .onTrue(new SetSelectedPosition(m_ScorePositionSubsystem, StateEnum.CORAL_LEVEL_3));
                new JoystickButton(m_operatorController, XboxController.Button.kY.value)
                                .onTrue(new SetSelectedPosition(m_ScorePositionSubsystem, StateEnum.CORAL_LEVEL_4));

                new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value)
                                .onTrue(new SetScoreSideCoral(m_ScorePositionSubsystem, DriveSideEnum.RIGHT));
                new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value)
                                .onTrue(new SetScoreSideCoral(m_ScorePositionSubsystem, DriveSideEnum.LEFT));

                // ------------------- Move Manually ------------------------

                // new JoystickButton(m_driverController,
                // XboxController.Button.kLeftBumper.value) // move the elevator down manually
                // .whileTrue(new ElevatorCommand(m_elevatorSubsystem,
                // -ElevatorConstants.kElevatorSpeed));

                // new JoystickButton(m_driverController,
                // XboxController.Button.kRightBumper.value) // move the elevator up manually
                // .whileTrue(new ElevatorCommand(m_elevatorSubsystem,
                // ElevatorConstants.kElevatorSpeed));

                // // move the arm counter-clockwise
                // new JoystickButton(m_driverController, XboxController.Button.kX.value)
                // .whileTrue(new MoveArm(m_armSubsystem, ArmConstants.kArmSpeed));

                // // move the arm clockwise
                // new JoystickButton(m_driverController, XboxController.Button.kY.value)
                // .whileTrue(new MoveArm(m_armSubsystem, -ArmConstants.kArmSpeed));
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
