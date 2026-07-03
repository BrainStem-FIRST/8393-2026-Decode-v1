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
        if (Double.isNaN(radians) || Double.isInfinite(radians))
            throw new IllegalArgumentException("radians is not a valid number when creating new Angle2d");
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
    public Angle2d plus(Angle2d other) {
        return new Angle2d(this.radians + other.radians);
    }

    /**
     * Subtracts another rotation from this rotation.
     *
     * @param other rotation to subtract
     * @return normalized difference between the rotations
     */
    public Angle2d minus(Angle2d other) {
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
     * Gets the signed shortest angular distance from this rotation to another rotation.
     *
     * @param other target rotation
     * @return signed angular distance in radians
     */
    public double angularDistanceTo(Angle2d other) {
        return other.minus(this).radians;
    }

    /**
     * Checks whether two rotations are equal within an angular tolerance.
     *
     * @param other rotation to compare against
     * @param epsilonRadians maximum allowed angular difference in radians
     * @return {@code true} if the rotations are within the tolerance
     */
    public boolean epsilonEquals(Angle2d other, double epsilonRadians) {
        return Math.abs(this.minus(other).radians) <= epsilonRadians;
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
        return plus(other.minus(this).times(t));
    }

    public static Angle2d kZero = fromDegrees(0);
    public static Angle2d k90 = fromDegrees(90);
    public static Angle2d k180 = fromDegrees(180);
    public static Angle2d k270 = fromDegrees(270);
}
