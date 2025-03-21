package frc.utils.AprilUtils;

/**
 * The different april tags with coordinates and IDs
 * @author Nathan Sandvig
 */
public enum AprilTag 
{
    RED_LOADER_LEFT(1, new Point(657.37, 25.80), 58.50, new Angle(126 - 180, AngularUnit.DEGREES)),
    RED_LOADER_RIGHT(2, new Point(657.37, 291.20), 58.50, new Angle(234 - 180, AngularUnit.DEGREES)),
    RED_PROCESSOR(3, new Point(455.15, 317.15), 51.25, new Angle(270 - 180, AngularUnit.DEGREES)),
    RED_BARGE_RIGHT(4, new Point(365.20, 241.64), 73.54, new Angle(0 - 180, AngularUnit.DEGREES)),
    RED_BARGE_LEFT(5, new Point(365.20, 75.39), 73.54, new Angle(0 - 180, AngularUnit.DEGREES)),
    RED_REEF_NEAR_LEFT(6, new Point(530.49, 130.17), 12.13, new Angle(300 - 180, AngularUnit.DEGREES)),
    RED_REEF_NEAR_CENTER(7, new Point(546.87, 158.50), 12.13, new Angle(0 - 180, AngularUnit.DEGREES)),
    RED_REEF_NEAR_RIGHT(8, new Point(530.49, 186.83), 12.13, new Angle(60 - 180, AngularUnit.DEGREES)),
    RED_REEF_FAR_RIGHT(9, new Point(497.77, 186.83), 12.13, new Angle(120 - 180, AngularUnit.DEGREES)),
    RED_REEF_FAR_CENTER(10, new Point(33.51, 25.80), 12.13, new Angle(180 - 180, AngularUnit.DEGREES)),
    RED_REEF_FAR_LEFT(11, new Point(497.77, 130.17), 12.13, new Angle(240 - 180, AngularUnit.DEGREES)),
    BLUE_LOADER_RIGHT(12, new Point(33.51, 25.80), 58.50, new Angle(54 - 180, AngularUnit.DEGREES)),
    BLUE_LOADER_LEFT(13, new Point(33.51, 291.20), 58.50, new Angle(306 - 180, AngularUnit.DEGREES)),
    BLUE_BARGE_LEFT(14, new Point(325.68, 241.64), 73.54, new Angle(180 - 180, AngularUnit.DEGREES)),
    BLUE_BARGE_RIGHT(15, new Point(325.68, 75.39), 73.54, new Angle(180 - 180, AngularUnit.DEGREES)),
    BLUE_PROCESSOR(16, new Point(235.73, -0.15), 51.25, new Angle(90 - 180, AngularUnit.DEGREES)),
    BLUE_REEF_NEAR_RIGHT(17, new Point(160.39, 130.17), 12.13, new Angle(240 - 180, AngularUnit.DEGREES)),
    BLUE_REEF_NEAR_CENTER(18, new Point(144.00, 158.50), 12.13, new Angle(180 - 180, AngularUnit.DEGREES)),
    BLUE_REEF_NEAR_LEFT(19, new Point(160.39, 186.83), 12.13, new Angle(120 - 180, AngularUnit.DEGREES)),
    BLUE_REEF_FAR_LEFT(20, new Point(193.10, 186.83), 12.13, new Angle(60 - 180, AngularUnit.DEGREES)),
    BLUE_REEF_FAR_CENTER(21, new Point(209.49, 158.50), 12.13, new Angle(0 - 180, AngularUnit.DEGREES)),
    BLUE_REEF_FAR_RIGHT(22, new Point(193.10, 130.17), 12.13, new Angle(300 - 180, AngularUnit.DEGREES)),
    COUNT(0, null, 0, null);

    /**
     * The ID number of the april tag
     */
    public final int ID;

    /**
     * The position of the april tag
     */
    public final Point position;

    /**
     * The height of the april tag
     */
    public final double height;

    /**
     * The angle facing straight towards the april tag
     */
    public final Angle angle;

    /**
     * Construct an AprilTag
     * @param position The position of the april tag
     * @param angle The angle facing straight towards the april tag
     */
    private AprilTag(int ID, Point position, double height, Angle angle)
    {
        this.ID = ID;
        this.position = new Point(position);
        this.height = height;
        this.angle = new Angle(angle);
    }
}
