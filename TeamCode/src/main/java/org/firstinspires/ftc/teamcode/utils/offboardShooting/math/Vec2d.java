package org.firstinspires.ftc.teamcode.utils.offboardShooting.math;

/**
 * Immutable 2D vector with {@code x} and {@code y} components.
 */
public record Vec2d(double x, double y) {
    public static final Vec2d kZero = Vec2d.kZero;

    /**
     * Adds another vector to this vector component-wise.
     *
     * @param v vector to add
     * @return sum of both vectors
     */
    public Vec2d plus(Vec2d v) {
        return new Vec2d(this.x + v.x, this.y + v.y);
    }

    /**
     * Subtracts another vector from this vector component-wise.
     *
     * @param v vector to subtract
     * @return difference between the vectors
     */
    public Vec2d minus(Vec2d v) {
        return new Vec2d(this.x - v.x, this.y - v.y);
    }

    /**
     * Multiplies this vector by a scalar.
     *
     * @param n scalar multiplier
     * @return scaled vector
     */
    public Vec2d times(double n) {
        return new Vec2d(this.x * n, this.y * n);
    }

    /**
     * Divides this vector by a scalar.
     *
     * @param n scalar divisor
     * @return divided vector
     */
    public Vec2d div(double n) {
        return new Vec2d(this.x / n, this.y / n);
    }

    /**
     * Converts this vector to a 3D vector in the XY plane.
     *
     * @return vector with the same x and y components and z equal to 0
     */
    public Vec3d to3D() {
        return new Vec3d(x, y, 0);
    }

    /**
     * Gets the magnitude, or Euclidean length, of this vector.
     *
     * @return vector magnitude
     */
    public double norm() {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Gets the squared magnitude of this vector.
     *
     * @return squared vector magnitude
     */
    public double normSqrd() {
        return x * x + y * y;
    }

    /**
     * Gets a perpendicular vector rotated 90 degrees counterclockwise.
     *
     * @return vector {@code (-y, x)}
     */
    public Vec2d perp() {
        return new Vec2d(-y, x);
    }

    public Vec2d unitVector() {
        return div(norm());
    }

    public Angle2d angle() {
        return Angle2d.fromRadians(Math.atan2(y, x));
    }
    /**
     * Rotates a vector by this rotation.
     *
     * @param angle angle to rotate vector by
     * @return rotated vector
     */
    public Vec2d rotate(Angle2d angle) {
        double cos = angle.cos();
        double sin = angle.sin();

        return new Vec2d(
                x * cos - y * sin,
                x * sin + y * cos
        );
    }
    public double dist(Vec2d other) {
        return minus(other).norm();
    }
    public double distSqrd(Vec2d other) {
        return minus(other).normSqrd();
    }
}
