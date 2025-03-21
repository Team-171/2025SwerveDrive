package frc.utils.AprilUtils;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * A 2d point
 * @author Nathan Sandvig
 */
public class Point
{
    /**
     * The x-coordinate of the point
     */
    private double x;

    /**
     * The y-coordinate of the point
     */
    private double y;

    /**
     * Default constructor for Point (0, 0)
     */
    public Point()
    {
        x = 0;
        y = 0;
    }

    /**
     * Create a point
     * @param x The x-coordinate of the point
     * @param y The y-coordinate of the point
     */
    public Point(double x, double y)
    {
        this.x = x;
        this.y = y;
    }

    /**
     * Create a point
     * @param copy The point to copy
     */
    public Point(Point copy)
    {
        x = copy.getX();
        y = copy.getY();
    }

    /**
     * Gets the x-coordinate of the point
     * @return The x-coordinate of the point
     */
    public double getX()
    {
        return x;
    }

    /**
     * Sets the x-coordinate of the point
     * @param x The x-coordinate of the point
     */
    public void setX(double x)
    {
        this.x = x;
    }

    /**
     * Gets the y-coordinate of the point
     * @return The y-coordinate of the point
     */
    public double getY()
    {
        return y;
    }

    /**
     * Sets the y-coordinate of the point
     * @param y The y-coordinate of the point
     */
    public void setY(double y)
    {
        this.y = y;
    }

    /**
     * Adds another point to this point
     * @param point The point being added to this point
     * @return The sum of the two points as a new point
     */
    public Point add(Point point)
    {
        return new Point(x + point.getX(), y + point.getY());
    }

    /**
     * Subtracts another point from this point
     * @param point The point being subtracted from this point
     * @return The difference of the two points as a new point
     */
    public Point subtract(Point point)
    {
        return new Point(x - point.getX(), y - point.getY());
    }

    /**
     * Multiplies this point by a scalar value
     * @param multiplier The scalar value to multiply this point by
     * @return The product of this point and the scalar value
     */
    public Point multiply(double multiplier)
    {
        return new Point(x * multiplier, y * multiplier);
    }

    /**
     * Divides this point by a scalar value
     * If the value is 0, returns (0, 0) to avoid throwing an exception
     * @param divisor The scalar value to divide this point by
     * @return The quotient of this point and the scalar value
     */
    public Point divide(double divisor)
    {
        Point quotient = new Point();
        if (divisor != 0)
        {
            quotient.setX(x / divisor);
            quotient.setY(y / divisor);
        }
        return quotient;
    }
}
