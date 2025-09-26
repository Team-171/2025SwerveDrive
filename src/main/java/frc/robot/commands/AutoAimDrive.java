// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ScorePositionSubsystem;
import frc.utils.LimelightHelpers;
import frc.utils.AprilUtils.Angle;
import frc.utils.AprilUtils.AngularUnit;
import frc.utils.AprilUtils.AprilTag;
import frc.utils.AprilUtils.AprilTagPosition;
import frc.utils.AprilUtils.Point;

/** An example command that uses an example subsystem. */
public class AutoAimDrive extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_driveSubsystem;
    private final ScorePositionSubsystem m_ScorePositionSubsystem;
    private static boolean enabled = true;
    private static AprilTagPosition aprilTagLeft = new AprilTagPosition.AprilTagPositionBuilder()
            .withLimelightX(LimelightConstants.leftXOffset)
            .withLimelightY(LimelightConstants.leftYOffset)
            .withLimelightHeight(LimelightConstants.leftHeight)
            .withLimelightHorizontalAngle(LimelightConstants.leftHorizontalAngle)
            .withLimelightVerticalAngle(LimelightConstants.leftVerticalAngle)
            .build();
    private static AprilTagPosition aprilTagRight = new AprilTagPosition.AprilTagPositionBuilder()
            .withLimelightX(LimelightConstants.rightXOffset)
            .withLimelightY(LimelightConstants.rightYOffset)
            .withLimelightHeight(LimelightConstants.rightHeight)
            .withLimelightHorizontalAngle(LimelightConstants.rightHorizontalAngle)
            .withLimelightVerticalAngle(LimelightConstants.rightVerticalAngle)
            .build();

    private boolean tagFound = false;
    private Point target = null;
    private double targetAngle;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutoAimDrive(DriveSubsystem subsystem, ScorePositionSubsystem m_ScorePositionSubsystem, boolean enabled) {
        m_driveSubsystem = subsystem;
        this.m_ScorePositionSubsystem = m_ScorePositionSubsystem;
        this.enabled = enabled;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    private Point calculateRobotPosition(NetworkTable frame, AprilTagPosition aprilTagPosition) {
        Point robotPosition = null;
        boolean seesTag = frame.getEntry("tv").getDouble(0.0) == 1.0;

        if (seesTag) {
            Double id = frame.getEntry("tid").getDouble(0.0);
            AprilTag aprilTag = findAprilId(id.intValue());
            if (aprilTag != null) {
                robotPosition = aprilTagPosition.calculatePosition(aprilTag,
                        new Angle(-frame.getEntry("tx").getDouble(0.0), AngularUnit.DEGREES),
                        new Angle(frame.getEntry("ty").getDouble(0.0), AngularUnit.DEGREES),
                        new Angle(m_driveSubsystem.getHeading(), AngularUnit.DEGREES));
            }
        }

        return robotPosition;
    }

    private AprilTag findAprilId(int id) {
        AprilTag aprilTag = null;
        if (id > 0 && id < AprilTag.COUNT.ordinal())
            aprilTag = AprilTag.values()[id - 1];
        return aprilTag;
    };

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (enabled) {
            Point robotPosition = null;
            NetworkTable leftLimelightFrame = LimelightHelpers.getLimelightNTTable(LimelightConstants.leftLimelight);
            NetworkTable rightLimelightFrame = LimelightHelpers.getLimelightNTTable(LimelightConstants.rightLimelight);
            Point leftPoint = calculateRobotPosition(leftLimelightFrame, aprilTagLeft);
            Point rightPoint = null;//calculateRobotPosition(rightLimelightFrame, aprilTagRight);
            if (rightPoint != null)
                robotPosition = rightPoint;
            else if (leftPoint != null)
                robotPosition = leftPoint;

            if (robotPosition != null) {
                tagFound = true;
                Translation2d robotPose2d = new Translation2d(Units.inchesToMeters(robotPosition.getX()),
                        Units.inchesToMeters(robotPosition.getY()));
                m_driveSubsystem.resetTranslation(robotPose2d);

                AprilTag aprilTag = null;
                Double leftID = leftLimelightFrame.getEntry("tid").getDouble(0.0);
                AprilTag leftAprilTag = findAprilId(leftID.intValue());
                Double rightID = rightLimelightFrame.getEntry("tid").getDouble(0.0);
                AprilTag rightAprilTag = findAprilId(rightID.intValue());

                if (leftAprilTag != null)
                    aprilTag = leftAprilTag;
                else if (rightAprilTag != null)
                    aprilTag = rightAprilTag;

                if (aprilTag != null) {
                    target = AprilTagPosition.calculateTargetPoint(aprilTag,
                            AutoAimConstants.autoAimXOffset,
                            AutoAimConstants.autoAimYOffset + m_ScorePositionSubsystem.getDriveSide().offsetY);

                    targetAngle = aprilTag.angle.getValue(AngularUnit.RADIANS);

                    SmartDashboard.putNumber("target X", Units.inchesToMeters(target.getX()));
                    SmartDashboard.putNumber("target Y", Units.inchesToMeters(target.getY()));
                }
            }
            if (tagFound) {
                m_driveSubsystem.driveToCartesianCoordinate(Units.inchesToMeters(target.getX()),
                        Units.inchesToMeters(target.getY()),
                        targetAngle);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
