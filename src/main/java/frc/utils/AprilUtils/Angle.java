package frc.utils.AprilUtils;

/**
 * A class used to contain angles with specified units
 * @author Nathan Sandvig
 */
public class Angle
{
    /**
     * Conversion factor between radians and degrees
     */
    private static final double RADIANS_PER_DEGREE = Math.PI / 180;

    /**
     * The angle's value in radians
     */
    private double radians;

    /**
     * Create a new angle
     * @param value The value of the angle in the specified units
     * @param units The units of the input value
     */
    public Angle(double value, AngularUnit units)
    {
        radians = switch (units) {
            case DEGREES -> value * RADIANS_PER_DEGREE;
            case RADIANS -> value;
        };
    }

    /**
     * Create a new angle from a copy angle
     * @param copy The angle being copied
     */
    public Angle(Angle copy)
    {
        radians = copy.radians;
    }

    /**
     * Get the value of the angle in the specified units
     * @param units The units of the return value
     * @return The value of this angle in the specified units
     */
    public final double getValue(AngularUnit units)
    {
        return switch (units) {
            case DEGREES -> radians / RADIANS_PER_DEGREE;
            case RADIANS -> radians;
        };
    }

    /**
     * Set the value of the angle
     * @param value The value of the angle in the specified units
     * @param units The units of the input value
     */
    public void setValue(double value, AngularUnit units)
    {
        radians = switch (units) {
            case DEGREES -> value * RADIANS_PER_DEGREE;
            case RADIANS -> value;
        };
    }
}
