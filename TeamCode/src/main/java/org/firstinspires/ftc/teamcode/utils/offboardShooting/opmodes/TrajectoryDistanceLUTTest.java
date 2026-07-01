package org.firstinspires.ftc.teamcode.utils.offboardShooting.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.shootingSystem.SRSHub;
import org.firstinspires.ftc.teamcode.robot.shootingSystem.Turret;
import org.firstinspires.ftc.teamcode.robot.shootingSystem.hood.HoodV2;
import org.firstinspires.ftc.teamcode.robot.shootingSystem.shooter.ShooterV2;
import org.firstinspires.ftc.teamcode.robot.subsystems.Collector;
import org.firstinspires.ftc.teamcode.utils.math.OdoInfo;
import org.firstinspires.ftc.teamcode.utils.misc.BatteryVoltageFilter;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories.TrajectoryDistanceLUT;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories.TrajectoryLoader;
import org.firstinspires.ftc.teamcode.utils.shootingMath.Vector3d;

import java.text.DecimalFormat;
import java.util.ArrayList;

@TeleOp(name="TrajectoryDistanceLUTTest", group="OffboardShooting")
@Config
public class TrajectoryDistanceLUTTest extends OpMode {
    public static double metersFromGoal = 1;
    private static final DecimalFormat df = new DecimalFormat("0.000");

    public enum ControlType {
        EXIT_SPEED,
        EXIT_ANGLE,
        IMPACT_ANGLE,
        OPTIMAL
    }
    public static boolean setShooterHoodToTrajectory = false;
    public static boolean runIntake = false;
    public static boolean engageClutch = false;
    public static ControlType controlType = ControlType.OPTIMAL;
    public static double exitSpeedMps, exitAngleDeg, impactAngleDeg;
    private TrajectoryDistanceLUT trajectoryDistanceLUT;
    private SRSHub srsHub;
    private ShooterV2 shooter;
    private HoodV2 hood;
    private Turret turret;
    private Collector collector;
    private BatteryVoltageFilter batteryVoltageFilter;
    private ElapsedTime timer;
    @Override
    public void init() {
        setShooterHoodToTrajectory = false;
        runIntake = false;
        engageClutch = false;
        reloadTrajectoryDistanceLUT();
        srsHub = new SRSHub(hardwareMap, telemetry);
        shooter = new ShooterV2(hardwareMap, telemetry, srsHub);
        ShooterV2.useBatteryVoltage = true;
        hood = new HoodV2(hardwareMap, telemetry, srsHub);
        turret = new Turret(hardwareMap, telemetry);
        collector = new Collector(hardwareMap, telemetry);
        batteryVoltageFilter = BatteryVoltageFilter.getInstance(hardwareMap);
        timer = new ElapsedTime();
        timer.reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        updateDriver1Controls();
        updateCollectorState();

        telemetry.addData("SetShooterHoodToTrajectory", setShooterHoodToTrajectory);
        telemetry.addData("RunIntake", runIntake);
        telemetry.addData("EngageClutch", engageClutch);
        telemetry.addLine();

        batteryVoltageFilter.update();
        srsHub.update();

        double dt = timer.seconds();
        timer.reset();

        collector.updateState(true);

        shooter.updateProperties();
        turret.updateProperties(dt);

        turret.controlTurretToTarget(
                0, 0, 0,
                0, 12, 0,
                new OdoInfo(0, 0, 0),
                batteryVoltageFilter.getVoltage()
        );

        Trajectory trajectory = chooseTrajectory(controlType);

        telemetry.addData("Control Type", controlType.name());
        telemetry.addLine("Aiming " + metersFromGoal + " meters forward from exit position");
        telemetry.addLine("Aiming " + trajectoryDistanceLUT.getRelGoalHeight() + " meters up from exit position");
        telemetry.addLine();
        telemetry.addData("ShooterSpeedRange", df.format(trajectoryDistanceLUT.getMinExitSpeedMps(metersFromGoal)) + "-" + df.format(trajectoryDistanceLUT.getMaxExitSpeedMps(metersFromGoal)));
        telemetry.addData("ExitAngleRange", df.format(Math.toDegrees(trajectoryDistanceLUT.getMinExitAngleRad(metersFromGoal))) + "-" + df.format(Math.toDegrees(trajectoryDistanceLUT.getMaxExitAngleRad(metersFromGoal))));
        telemetry.addData("ImpactAngleRange", df.format(Math.toDegrees(trajectoryDistanceLUT.getMinImpactAngleRad(metersFromGoal))) + "-" + df.format(Math.toDegrees(trajectoryDistanceLUT.getMaxImpactAngleRad(metersFromGoal))));
        telemetry.addLine();
        telemetry.addData("Chosen Trajectory", trajectory);
        telemetry.addLine();
        if (controlType == ControlType.EXIT_SPEED)
            telemetry.addData("ShooterSpeedInRange", trajectoryDistanceLUT.exitSpeedInRange(metersFromGoal, exitSpeedMps));
        if (controlType == ControlType.EXIT_ANGLE)
            telemetry.addData("ExitAngleInRange", trajectoryDistanceLUT.exitAngleInRange(metersFromGoal, Math.toRadians(exitAngleDeg)));
        if (controlType == ControlType.IMPACT_ANGLE)
            telemetry.addData("ImpactAngleInRange", trajectoryDistanceLUT.impactAngleInRange(metersFromGoal, Math.toRadians(impactAngleDeg)));

        telemetry.addLine();
        telemetry.addData("shooterSpeedActualTps", shooter.getVelTps());
        if (trajectory != null) {
            double shooterSpeedTps = ShooterV2.params.getTpsFunction.apply(trajectory.exitSpeedMps);
            telemetry.addData("shooterSpeedTargetTps", shooterSpeedTps);
            if (setShooterHoodToTrajectory) {
                shooter.setShooterVelocityPID(shooterSpeedTps, batteryVoltageFilter.getVoltage());
                hood.setTargetExitAngle(trajectory.exitAngleRad);
            }

            ArrayList<Vector3d> points = trajectory.simulateTrajectory(300, 10, 0, new Vector3d(0, 0, 0), new Vector3d(0, 0, 0));
            Vector3d startingPositionMeters = points.get(0);
            Vector3d startingPositionFeet = startingPositionMeters.times(3.281);
            Vector3d landingPositionMeters = points.get(points.size() - 1);
            Vector3d landingPositionFeet = landingPositionMeters.times(3.281);

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setFill("green");
            packet.fieldOverlay().fillCircle(startingPositionFeet.x, startingPositionFeet.y, 1);
            packet.fieldOverlay().setFill("red");
            packet.fieldOverlay().fillCircle(landingPositionFeet.x, landingPositionFeet.y, 1);

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
        if (!setShooterHoodToTrajectory) {
            shooter.stopMotor();
        }

        telemetry.update();
    }

    private void reloadTrajectoryDistanceLUT() {
        trajectoryDistanceLUT = TrajectoryLoader.loadTrajectoryDistanceLUT("mtiTrajectories.json");
    }

    private Trajectory chooseTrajectory(ControlType controlType) {
        return switch (controlType) {
            case EXIT_SPEED ->
                    trajectoryDistanceLUT.getInterpolatedExitSpeedTrajectory(metersFromGoal, exitSpeedMps);
            case EXIT_ANGLE ->
                    trajectoryDistanceLUT.getInterpolatedExitAngleTrajectory(metersFromGoal, Math.toRadians(exitAngleDeg));
            case IMPACT_ANGLE ->
                    trajectoryDistanceLUT.getInterpolatedImpactAngleTrajectory(metersFromGoal, Math.toRadians(impactAngleDeg));
            case OPTIMAL -> trajectoryDistanceLUT.getInterpolatedOptimalTrajectory(metersFromGoal);
        };
    }

    private void updateDriver1Controls() {
        if (gamepad1.yWasPressed())
            setShooterHoodToTrajectory = !setShooterHoodToTrajectory;
        if (gamepad1.bWasPressed()) {
            engageClutch = !engageClutch;
            if (engageClutch)
                runIntake = false;
        }
        if (!engageClutch)
            runIntake = gamepad1.right_trigger > 0.3;
        else if (gamepad1.aWasPressed())
                runIntake = !runIntake;
    }

    private void updateCollectorState() {
        if (runIntake)
            collector.setIntakeState(Collector.IntakeState.INTAKE);
        else
            collector.setIntakeState(Collector.IntakeState.OFF);

        if (engageClutch)
            collector.setClutchState(Collector.ClutchState.ENGAGED);
        else
            collector.setClutchState(Collector.ClutchState.DISENGAGED);
    }
}
