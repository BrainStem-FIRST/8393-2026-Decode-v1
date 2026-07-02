package org.firstinspires.ftc.teamcode.utils.offboardShooting.math;

public record Vector2d(double x, double y) {

    public Vector2d add(Vector2d v) {
        return new Vector2d(this.x + v.x, this.y + v.y);
    }

    public Vector2d sub(Vector2d v) {
        return new Vector2d(this.x - v.x, this.y - v.y);
    }

    public Vector2d times(double n) {
        return new Vector2d(this.x * n, this.y * n);
    }

    public Vector2d div(double n) {
        return new Vector2d(this.x / n, this.y / n);
    }

    public Vector3d to3D() {
        return new Vector3d(x, y, 0);
    }

    public double mag() {
        return Math.sqrt(x * x + y * y);
    }

    public double magSqrd() {
        return x * x + y * y;
    }

    public Vector2d perp() {
        return new Vector2d(-y, x);
    }

}