// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.helperObjects.LimelightEnums;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.LimelightHelpers;

/** An example command that uses an example subsystem. */
public class AutoAimCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem driveSubsystem;

    private double desiredDistance;
    private double desiredHeading;
    private boolean isFinished;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutoAimCommand(DriveSubsystem subsystem) {
        driveSubsystem = subsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isFinished = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double leftId = LimelightHelpers.getFiducialID(LimelightConstants.leftLimelight);
        double rightId = LimelightHelpers.getFiducialID(LimelightConstants.rightLimelight);
        double id = -1;
        LimelightEnums bestLimelight = LimelightEnums.NEITHER;

        if (leftId != -1 && rightId != -1) {
            if (leftId == rightId) {
                id = rightId;
                bestLimelight = LimelightEnums.RIGHT;
            } else {
                Pose3d leftTargetPose = LimelightHelpers.getTargetPose3d_RobotSpace(LimelightConstants.leftLimelight);
                double leftTargetX = leftTargetPose.getX();
                double leftTargetY = leftTargetPose.getY();
                double leftDistance = Math.hypot(leftTargetX, leftTargetY);
                Pose3d rightTargetPose = LimelightHelpers.getTargetPose3d_RobotSpace(LimelightConstants.rightLimelight);
                double rightTargetX = rightTargetPose.getX();
                double rightTargetY = rightTargetPose.getY();
                double rightDistance = Math.hypot(rightTargetX, rightTargetY);

                id = leftDistance < rightDistance ? leftId : rightId;
                bestLimelight = leftDistance < rightDistance ? LimelightEnums.LEFT : LimelightEnums.RIGHT;
            }
        } else if (leftId != -1) {
            id = leftId;
            bestLimelight = LimelightEnums.LEFT;
        } else if (rightId != -1) {
            id = rightId;
            bestLimelight = LimelightEnums.RIGHT;
        } else {
            id = -1;
            bestLimelight = LimelightEnums.NEITHER;
        }

        double desiredDirection = 0;
        desiredHeading = AprilTagIdToAngle(id);
        desiredDistance = 0;

        if (bestLimelight == LimelightEnums.LEFT) {
            Pose3d targetPose = LimelightHelpers.getTargetPose3d_RobotSpace(LimelightConstants.leftLimelight);
            double targetX = targetPose.getX();
            double targetY = targetPose.getY();
            desiredDirection = Math.atan2(targetY, targetX);
            desiredDistance = Math.hypot(targetX, targetY);
        }
        if (bestLimelight == LimelightEnums.RIGHT) {
            Pose3d targetPose = LimelightHelpers.getTargetPose3d_RobotSpace(LimelightConstants.rightLimelight);
            double targetX = targetPose.getX();
            double targetY = targetPose.getY();
            desiredDirection = Math.atan2(targetY, targetX);
            desiredDistance = Math.hypot(targetX, targetY);
        }
        if (bestLimelight != LimelightEnums.NEITHER) {
            driveSubsystem.driveToDesired(desiredDistance, desiredDirection, desiredHeading);
        }
    }

    /**
     * Gives desired heading based on the AprilTag id on field.
     * 
     * @param id The AprilTag ID
     * @return Desired angle in radians :)
     */
    private double AprilTagIdToAngle(double id) {
        double radianAngle;
        switch (((int) id)) {
            case 18:
            case 7:
                radianAngle = Math.toRadians(0);
                break;
            case 19:
            case 6:
                radianAngle = Math.toRadians(60);
                break;
            case 20:
            case 11:
                radianAngle = Math.toRadians(120);
                break;
            case 21:
            case 10:
                radianAngle = Math.toRadians(180);
                break;
            case 22:
            case 9:
                radianAngle = Math.toRadians(-120);
                break;
            case 17:
            case 8:
                radianAngle = Math.toRadians(-60);
                break;
            default:
                radianAngle = Math.toRadians(driveSubsystem.getHeading());
                break;
        }
        return radianAngle;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double errorHeading = desiredHeading - Math.toRadians(driveSubsystem.getHeading());

        while (errorHeading > Math.PI) {
          errorHeading -= (2 * Math.PI);
        }
        while (errorHeading < -Math.PI) {
          errorHeading += (2 * Math.PI);
        }

        if (0 < desiredDistance && desiredDistance < 0.1 && 0 < errorHeading && errorHeading < Math.toRadians(10)) {
            isFinished = true;
        }
        return isFinished;
    }
}
