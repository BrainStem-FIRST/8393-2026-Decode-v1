package org.firstinspires.ftc.teamcode.utils.offboardShooting.math;

/**
 * Immutable 2D rotation stored in radians.
 *
 * <p>All rotations are normalized to the range {@code (-pi, pi]} when created.
 */
public record Angle2d(double radians) {
    /**
     * Creates a rotation from an angle in radians.
     *
     * @param radians angle in radians
     */
    public Angle2d {
        radians = normalizeRadians(radians);
    }

    /**
     * Creates a rotation from an angle in radians.
     *
     * @param radians angle in radians
     * @return normalized rotation
     */
    public static Angle2d fromRadians(double radians) {
        return new Angle2d(radians);
    }

    /**
     * Creates a rotation from an angle in degrees.
     *
     * @param degrees angle in degrees
     * @return normalized rotation
     */
    public static Angle2d fromDegrees(double degrees) {
        return new Angle2d(Math.toRadians(degrees));
    }

    /**
     * Creates the rotation pointing in the same direction as a 2D vector.
     *
     * @param v vector whose direction is used
     * @return rotation with heading {@code atan2(v.y(), v.x())}
     */
    public static Angle2d fromVector(Vec2d v) {
        return new Angle2d(Math.atan2(v.y(), v.x()));
    }

    /**
     * Gets this rotation in degrees.
     *
     * @return angle in degrees
     */
    public double degrees() {
        return Math.toDegrees(radians);
    }

    /**
     * Gets the cosine of this rotation.
     *
     * @return cosine of the stored angle
     */
    public double cos() {
        return Math.cos(radians);
    }

    /**
     * Gets the sine of this rotation.
     *
     * @return sine of the stored angle
     */
    public double sin() {
        return Math.sin(radians);
    }

    /**
     * Gets the tangent of this rotation.
     *
     * @return tangent of the stored angle
     */
    public double tan() {
        return Math.tan(radians);
    }

    /**
     * Adds another rotation to this rotation.
     *
     * @param other rotation to add
     * @return normalized sum of both rotations
     */
    public Angle2d add(Angle2d other) {
        return new Angle2d(this.radians + other.radians);
    }

    /**
     * Subtracts another rotation from this rotation.
     *
     * @param other rotation to subtract
     * @return normalized difference between the rotations
     */
    public Angle2d sub(Angle2d other) {
        return new Angle2d(this.radians - other.radians);
    }

    /**
     * Scales this rotation by a scalar.
     *
     * @param scalar value to multiply the angle by
     * @return normalized scaled rotation
     */
    public Angle2d times(double scalar) {
        return new Angle2d(this.radians * scalar);
    }

    /**
     * Divides this rotation by a scalar.
     *
     * @param scalar value to divide the angle by
     * @return normalized divided rotation
     */
    public Angle2d div(double scalar) {
        return new Angle2d(this.radians / scalar);
    }

    /**
     * Gets the opposite rotation.
     *
     * @return rotation with the negated angle
     */
    public Angle2d inverse() {
        return new Angle2d(-radians);
    }

    /**
     * Converts this rotation to a unit vector.
     *
     * @return vector with x equal to cosine and y equal to sine
     */
    public Vec2d toUnitVector() {
        return new Vec2d(cos(), sin());
    }

    /**
     * Rotates a vector by this rotation.
     *
     * @param v vector to rotate
     * @return rotated vector
     */
    public Vec2d rotate(Vec2d v) {
        double cos = cos();
        double sin = sin();

        return new Vec2d(
                v.x() * cos - v.y() * sin,
                v.x() * sin + v.y() * cos
        );
    }

    /**
     * Gets the signed shortest angular distance from this rotation to another rotation.
     *
     * @param other target rotation
     * @return signed angular distance in radians
     */
    public double angularDistanceTo(Angle2d other) {
        return other.sub(this).radians;
    }

    /**
     * Checks whether two rotations are equal within an angular tolerance.
     *
     * @param other rotation to compare against
     * @param epsilonRadians maximum allowed angular difference in radians
     * @return {@code true} if the rotations are within the tolerance
     */
    public boolean epsilonEquals(Angle2d other, double epsilonRadians) {
        return Math.abs(this.sub(other).radians) <= epsilonRadians;
    }

    /**
     * Normalizes an angle to the range {@code (-pi, pi]}.
     *
     * @param radians angle in radians
     * @return normalized angle in radians
     */
    private static double normalizeRadians(double radians) {
        double angle = radians % (2.0 * Math.PI);

        if (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        } else if (angle <= -Math.PI) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    /**
     * Linearly interpolates from this rotation to another rotation.
     *
     * @param other target rotation
     * @param t interpolation parameter, where 0 returns this rotation and 1 returns {@code other}
     * @return interpolated rotation
     */
    public Angle2d lerp(Angle2d other, double t) {
        return add(other.sub(this).times(t));
    }
}
