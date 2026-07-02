package org.firstinspires.ftc.teamcode.utils.offboardShooting.math;

/**
 * Immutable 3D vector with {@code x}, {@code y}, and {@code z} components.
 */
public record Vec3d(double x, double y, double z) {

  /**
   * Adds another vector to this vector component-wise.
   *
   * @param v vector to add
   * @return sum of both vectors
   */
  public Vec3d add(Vec3d v) {
    return new Vec3d(this.x + v.x, this.y + v.y, this.z + v.z);
  }

  /**
   * Subtracts another vector from this vector component-wise.
   *
   * @param v vector to subtract
   * @return difference between the vectors
   */
  public Vec3d sub(Vec3d v) {
    return new Vec3d(this.x - v.x, this.y - v.y, this.z - v.z);
  }

  /**
   * Multiplies this vector by a scalar.
   *
   * @param n scalar multiplier
   * @return scaled vector
   */
  public Vec3d times(double n) {
    return new Vec3d(this.x * n, this.y * n, this.z * n);
  }

  /**
   * Divides this vector by a scalar.
   *
   * @param n scalar divisor
   * @return divided vector
   */
  public Vec3d div(double n) {
    return new Vec3d(this.x / n, this.y / n, this.z / n);
  }

  /**
   * Projects this vector onto the XY plane.
   *
   * @return vector with the same x and y components and z equal to 0
   */
  public Vec3d to2D() {
    return new Vec3d(x, y, 0);
  }

  /**
   * Gets the magnitude, or Euclidean length, of this vector.
   *
   * @return vector magnitude
   */
  public double mag() {
    return Math.sqrt(x * x + y * y + z * z);
  }

  /**
   * Gets the squared magnitude of this vector.
   *
   * @return squared vector magnitude
   */
  public double magSqrd() {
    return x * x + y * y + z * z;
  }

  /**
   * Gets a perpendicular vector in the XY plane rotated 90 degrees counterclockwise.
   *
   * @return vector {@code (-y, x, 0)}
   */
  public Vec3d perpInXY() {
    return new Vec3d(-y, x, 0);
  }

}
