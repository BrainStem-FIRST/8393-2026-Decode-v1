package org.firstinspires.ftc.teamcode.utils.shootingMath;

import org.firstinspires.ftc.teamcode.utils.offboardShooting.math.Vec3d;

public class AnswerKeyPt1 {
    public final boolean solutionExists;
    public final LaunchVector launchVector;
    
    public final Vec3d lookAheadExitPos;
    public final Vec3d lookAheadVelAtExitPos;
    public final Vec3d currentVelAtExitPos;

    public AnswerKeyPt1(Vec3d exitPos, Vec3d lookAheadVelAtExitPos, Vec3d currentVelAtExitPos) {
        solutionExists = false;
        launchVector = null;
        this.lookAheadExitPos = exitPos;
        this.lookAheadVelAtExitPos = lookAheadVelAtExitPos;
        this.currentVelAtExitPos = currentVelAtExitPos;
    }
    public AnswerKeyPt1(LaunchVector launchVector, Vec3d lookAheadExitPos, Vec3d lookAheadVelAtExitPos, Vec3d currentVelAtExitPos) {
        solutionExists = true;
        this.launchVector = launchVector;
        this.lookAheadExitPos = lookAheadExitPos;
        this.lookAheadVelAtExitPos = lookAheadVelAtExitPos;
        this.currentVelAtExitPos = currentVelAtExitPos;
    }

    public String toString() {
        return "" + launchVector;
    }
}
