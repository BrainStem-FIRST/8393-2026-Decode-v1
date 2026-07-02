package org.firstinspires.ftc.teamcode.utils.offboardShooting.math;

public record Rotation2d(double radians) {

    public Rotation2d {
        radians = normalizeRadians(radians);
    }

    public static Rotation2d fromRadians(double radians) {
        return new Rotation2d(radians);
    }

    public static Rotation2d fromDegrees(double degrees) {
        return new Rotation2d(Math.toRadians(degrees));
    }

    public static Rotation2d fromVector(Vector2d v) {
        return new Rotation2d(Math.atan2(v.y(), v.x()));
    }

    public double degrees() {
        return Math.toDegrees(radians);
    }

    public double cos() {
        return Math.cos(radians);
    }

    public double sin() {
        return Math.sin(radians);
    }

    public double tan() {
        return Math.tan(radians);
    }

    public Rotation2d add(Rotation2d other) {
        return new Rotation2d(this.radians + other.radians);
    }

    public Rotation2d sub(Rotation2d other) {
        return new Rotation2d(this.radians - other.radians);
    }

    public Rotation2d times(double scalar) {
        return new Rotation2d(this.radians * scalar);
    }

    public Rotation2d div(double scalar) {
        return new Rotation2d(this.radians / scalar);
    }

    public Rotation2d inverse() {
        return new Rotation2d(-radians);
    }

    public Vector2d toUnitVector() {
        return new Vector2d(cos(), sin());
    }

    public Vector2d rotate(Vector2d v) {
        double cos = cos();
        double sin = sin();

        return new Vector2d(
                v.x() * cos - v.y() * sin,
                v.x() * sin + v.y() * cos
        );
    }

    public double angularDistanceTo(Rotation2d other) {
        return other.sub(this).radians;
    }

    public boolean epsilonEquals(Rotation2d other, double epsilonRadians) {
        return Math.abs(this.sub(other).radians) <= epsilonRadians;
    }

    private static double normalizeRadians(double radians) {
        double angle = radians % (2.0 * Math.PI);

        if (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        } else if (angle <= -Math.PI) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }
}