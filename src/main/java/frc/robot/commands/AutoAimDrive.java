// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AutoAimDrive extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_driveSubsystem;
    private final ScorePositionSubsystem m_ScorePositionSubsystem;
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

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutoAimDrive(DriveSubsystem subsystem, ScorePositionSubsystem m_ScorePositionSubsystem, int id) {
        m_driveSubsystem = subsystem;
        this.m_ScorePositionSubsystem = m_ScorePositionSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    private Point calculateRobotPosition(String limelight) {
        Point robotPosition = null;
        boolean seesTag = LimelightHelpers.getTV(limelight);

        if (seesTag) {
            AprilTag aprilTag = findAprilId(limelight);
            if (aprilTag != null) {
                robotPosition = aprilTagLeft.calculatePosition(aprilTag,
                        new Angle(LimelightHelpers.getTX(limelight), AngularUnit.DEGREES),
                        new Angle(LimelightHelpers.getTY(limelight), AngularUnit.DEGREES),
                        new Angle(m_driveSubsystem.getHeading(), AngularUnit.DEGREES));
            }
        }

        return robotPosition;
    }

    private AprilTag findAprilId(String limelight) {
        int id = (int) LimelightHelpers.getFiducialID(limelight);
        AprilTag aprilTag = null;
        for (int i = 0; i < AprilTag.COUNT.ordinal(); ++i)
            if (AprilTag.values()[i].ID == id)
                aprilTag = AprilTag.values()[i];
        return aprilTag;
    };

    // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Point robotPosition = null;
    Point leftPoint = calculateRobotPosition(LimelightConstants.leftLimelight);
    Point rightPoint = calculateRobotPosition(LimelightConstants.rightLimelight);
    if (leftPoint != null)
        robotPosition = leftPoint;
    else if (rightPoint != null)
        robotPosition = rightPoint;

    if (robotPosition != null)
    {
        Pose2d robotPose2d = new Pose2d(robotPosition.getX(), robotPosition.getY(), new Rotation2d(m_driveSubsystem.getHeading()));
        m_driveSubsystem.resetOdometry(robotPose2d);

        AprilTag aprilTag = null;
        AprilTag leftAprilTag = findAprilId(LimelightConstants.leftLimelight);
        AprilTag rightAprilTag = findAprilId(LimelightConstants.rightLimelight);
        if (leftAprilTag != null)
            aprilTag = leftAprilTag;
        else if (rightAprilTag != null)
            aprilTag = rightAprilTag;
        
        if (aprilTag != null)
        {   
            Point target = AprilTagPosition.calculateTargetPoint(aprilTag,
                 AutoAimConstants.autoAimXOffset,
                 AutoAimConstants.autoAimYOffset + m_ScorePositionSubsystem.getDriveSide().offsetY);
            m_driveSubsystem.driveToCartesianCoordinate(target.getX(), target.getY(), aprilTag.angle.getValue(AngularUnit.RADIANS));
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
