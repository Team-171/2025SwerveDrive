package frc.utils.AprilUtils;

/**
 * Calculates the robot and target position from the april tag
 * @author Nathan Sandvig
 */
public class AprilTagPosition
{
    /**
     * The robot-relative x-offset of the limelight from center (x is positive forward)
     */
    private double limelightX = 0;

    /**
     * The robot-relative y-offset of the limelight from center (y is positive to the left)
     */
    private double limelightY = 0;

    /**
     * The height of the limelight from the ground (height is positive up)
     */
    private double limelightHeight = 0;

    /**
     * The robot-relative angular offset of the limelight from straight forward (angle is positive counter-clockwise)
     */
    private final Angle limelightHorizontalAngle = new Angle(0, AngularUnit.RADIANS);

    /**
     * The angle of the limelight from the ground (angle is positive upward, 0 is parallel to the ground)
     */
    private final Angle limelightVerticalAngle = new Angle(0, AngularUnit.RADIANS);

    /**
     * Private constructor for the builder
     * @param builder The builder
     */
    private AprilTagPosition(AprilTagPositionBuilder builder)
    {
        // Only set the private variable values if they were included in the build
        if (builder.limelightX != null)
            limelightX = builder.limelightX;
        if (builder.limelightY != null)
            limelightY = builder.limelightY;
        if (builder.limelightHeight != null)
            limelightHeight = builder.limelightHeight;
        if (builder.limelightHorizontalAngle != null)
            limelightHorizontalAngle.setValue(builder.limelightHorizontalAngle.getValue(AngularUnit.RADIANS), AngularUnit.RADIANS);
        if (builder.limelightVerticalAngle != null)
            limelightVerticalAngle.setValue(builder.limelightVerticalAngle.getValue(AngularUnit.RADIANS), AngularUnit.RADIANS);
    }

    /**
     * Calculate the distance to an april tag using its known height and measured vertical angle
     * @param limelightVertical The vertical angle to the april tag
     * @param aprilTagHeight The height of the tracked part of the april tag
     * @return The distance to the april tag
     */
    private double calculateDistance(Angle limelightVertical, double aprilTagHeight)
    {
        double heightDifference = aprilTagHeight - limelightHeight;
        double verticalAngle = limelightVerticalAngle.getValue(AngularUnit.RADIANS) +
                limelightVertical.getValue(AngularUnit.RADIANS);
        return heightDifference / Math.tan(verticalAngle);
    }

    /**
     * Calculate the position of the limelight
     * @param aprilTag The april tag
     * @param distance The distance to the april tag
     * @param limelightHorizontal The horizontal angle to the april tag as seen by the limelight
     * @param robotAngle The absolute angle of the robot
     * @return The april tag coordinate relative to the limelight
     */
    private Point calculateLimelightPosition(AprilTag aprilTag, double distance, Angle limelightHorizontal, Angle robotAngle)
    {
        double absoluteLimelightAngle = robotAngle.getValue(AngularUnit.RADIANS) 
            + limelightHorizontal.getValue(AngularUnit.RADIANS)
            + limelightHorizontalAngle.getValue(AngularUnit.RADIANS);
        double xOffset = distance * Math.cos(absoluteLimelightAngle);
        double yOffset = distance * Math.sin(absoluteLimelightAngle);
        double limelightX = aprilTag.position.getX() - xOffset;
        double limelightY = aprilTag.position.getY() - yOffset;
        return new Point(limelightX, limelightY);
    }

    /**
     * Calculates the offset of the limelight to the robot in april-tag-relative space
     * @param robotAngle The angle of the robot
     * @return The limelight coordinate relative to the robot
     */
    private Point calculateLimelightOffset(Angle robotAngle)
    {
        double robotAngleRadians = robotAngle.getValue(AngularUnit.RADIANS);
        double limelightOffsetX = (limelightX * Math.cos(robotAngleRadians)) - (limelightY * Math.sin(robotAngleRadians));
        double limelightOffsetY = (limelightY * Math.cos(robotAngleRadians)) + (limelightX * Math.sin(robotAngleRadians));
        return new Point(limelightOffsetX, limelightOffsetY);
    }

    /**
     * Calculate the position of the robot using an april tag
     * @param limelightHorizontal The horizontal angle of the april tag sensed by the limelight
     * @param limelightVertical The vertical angle of the april tag sensed by the limelight
     * @param robotAngle The absolute angle of the robot
     * @param aprilTagAngle The angle FACING STRAIGHT TOWARD the april tag
     * @param aprilTagHeight The height of the tracked part of the april tag
     * @return The april tag coordinate relative to the robot
     */
    public Point calculatePosition(AprilTag aprilTag, Angle limelightHorizontal, Angle limelightVertical, Angle robotAngle)
    {
        // Calculate the distance to target using the height, vertical offset, and vertical sensor value
        double distance = calculateDistance(limelightVertical, aprilTag.height);

        // Calculate the position of the april tag
        Point limelightPosition = calculateLimelightPosition(aprilTag, distance, limelightHorizontal, robotAngle);

        // Calculate the offset from robot to limelight using robot angle and limelight offsets
        Point limelightOffset = calculateLimelightOffset(robotAngle);

        // Subtract the offset from the position to calculate the robot position
        return limelightPosition.subtract(limelightOffset);
    }

    /**
     * Calculates a target point given an april tag and a local offset
     * @param aprilTag The april tag
     * @param offsetX The local x offset from the april tag
     * @param offsetY The local y offset from the april tag
     * @return The target coordinate in the field
     */
    public static Point calculateTargetPoint(AprilTag aprilTag, double offsetX, double offsetY)
    {
        double aprilTagAngleRadians = aprilTag.angle.getValue(AngularUnit.RADIANS);
        double xOffset = offsetX * Math.cos(aprilTagAngleRadians) - offsetY * Math.sin(aprilTagAngleRadians);
        double yOffset = offsetY * Math.cos(aprilTagAngleRadians) + offsetX * Math.sin(aprilTagAngleRadians);
        double targetX = aprilTag.position.getX() - xOffset;
        double targetY = aprilTag.position.getY() - yOffset;

        return new Point(targetX, targetY);
    }

    /**
     * Builder class for AprilTagPositionBuilder
     * @author Nathan Sandvig
     */
    public static class AprilTagPositionBuilder
    {
        /**
         * The robot-relative x-offset of the limelight from center (x is positive forward)
         */
        private Double limelightX = null;

        /**
         * The robot-relative y-offset of the limelight from center (y is positive to the left)
         */
        private Double limelightY = null;

        /**
         * The height of the limelight from the ground (height is positive up)
         */
        private Double limelightHeight = null;

        /**
         * The robot-relative angular offset of the limelight from straight forward (angle is positive counter-clockwise)
         */
        private Angle limelightHorizontalAngle = null;

        /**
         * The angle of the limelight from the ground (angle is positive upward, 0 is parallel to the ground)
         */
        private Angle limelightVerticalAngle = null;

        /**
         * Add a limelight x-offset to the build
         * @param limelightX The limelight x-offset
         * @return The builder for build chaining
         */
        public AprilTagPositionBuilder withLimelightX(double limelightX)
        {
            this.limelightX = limelightX;
            return this;
        }

        /**
         * Add a limelight y-offset to the build
         * @param limelightY The limelight y-offset
         * @return The builder for build chaining
         */
        public AprilTagPositionBuilder withLimelightY(double limelightY)
        {
            this.limelightY = limelightY;
            return this;
        }

        /**
         * Add a limelight height to the build
         * @param limelightHeight The limelight height
         * @return The builder for build chaining
         */
        public AprilTagPositionBuilder withLimelightHeight(double limelightHeight)
        {
            this.limelightHeight = limelightHeight;
            return this;
        }

        /**
         * Add a limelight horizontal angle to the build
         * @param limelightHorizontalAngle The limelight horizontal angle
         * @return The builder for build chaining
         */
        public AprilTagPositionBuilder withLimelightHorizontalAngle(Angle limelightHorizontalAngle)
        {
            this.limelightHorizontalAngle = new Angle(limelightHorizontalAngle);
            return this;
        }

        /**
         * Add a limelight vertical angle to the build
         * @param limelightVerticalAngle The limelight vertical angle
         * @return The builder for build chaining
         */
        public AprilTagPositionBuilder withLimelightVerticalAngle(Angle limelightVerticalAngle)
        {
            this.limelightVerticalAngle = limelightVerticalAngle;
            return this;
        }

        /**
         * Build the AprilTagOffsetCalculator
         * @return The new AprilTagOffsetCalculator
         */
        public AprilTagPosition build()
        {
            return new AprilTagPosition(this);
        }
    }
}
