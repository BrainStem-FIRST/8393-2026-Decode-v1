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
import org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories.TrajectoryMath;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories.TrajectoryWrapper;
import org.firstinspires.ftc.teamcode.utils.shootingMath.Vector3d;

import java.text.DecimalFormat;
import java.util.ArrayList;

@TeleOp(name="TrajectoryDistanceLUTTest", group="OffboardShooting")
@Config
public class TrajectoryDistanceLUTTest extends OpMode {
    public static double metersFromGoal = 3;
    private static final DecimalFormat df = new DecimalFormat("0.000");

    public enum ControlType {
        EXIT_SPEED,
        EXIT_ANGLE,
        OPTIMAL
    }
    public static boolean setShooterHoodToTrajectory = false;
    public static boolean useDynamicHood = true;
    public static boolean runIntake = false;
    public static boolean engageClutch = false;
    public static ControlType controlType = ControlType.EXIT_SPEED;
    public static double exitSpeedMps, exitAngleDeg;
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
        telemetry.setMsTransmissionInterval(20);

        telemetry.addLine("Ready");
        telemetry.addLine();
        TrajectoryDistanceLUT.NeighborTrajectoryInfo info = trajectoryDistanceLUT.getNeighboringTrajectoryLUTs(metersFromGoal);
        telemetry.addData("loLUT", info.loLUT());
        telemetry.addData("hiLUT", info.hiLUT());
        telemetry.addData("loLUT speed sorted", info.loLUT().getSpeedSortedTrajectoriesString());
        telemetry.addData("hiLUT speed sorted", info.hiLUT().getSpeedSortedTrajectoriesString());
        telemetry.update();
    }

    @Override
    public void loop() {
        if (controlType == ControlType.EXIT_ANGLE) {
            useDynamicHood = false;
            telemetry.addLine("Dynamic Hood disabled in " + controlType.name() + " control type");
            telemetry.addLine();
        }

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

        double curExitSpeedMps = ShooterV2.params.getMpsFunction.apply(shooter.getVelTps());

        TrajectoryWrapper targetTrajectory = chooseTrajectory(controlType);
        TrajectoryWrapper compensatedTrajectory = trajectoryDistanceLUT.getInterpolatedExitSpeedTrajectory(metersFromGoal, curExitSpeedMps, targetTrajectory.exitAngleRad);

        telemetry.addData("Control Type", controlType.name());
        telemetry.addLine("Aiming " + metersFromGoal + " meters forward from exit position");
        telemetry.addLine("Aiming " + trajectoryDistanceLUT.getRelGoalHeight() + " meters up from exit position");
        telemetry.addLine();
        telemetry.addData("ShooterSpeedRange", df.format(trajectoryDistanceLUT.getMinExitSpeedMps(metersFromGoal)) + "-" + df.format(trajectoryDistanceLUT.getMaxExitSpeedMps(metersFromGoal, targetTrajectory.exitAngleRad)));
        telemetry.addData("ExitAngleRange", df.format(Math.toDegrees(trajectoryDistanceLUT.getMinExitAngleRad(metersFromGoal))) + "-" + df.format(Math.toDegrees(trajectoryDistanceLUT.getMaxExitAngleRad(metersFromGoal))));
        telemetry.addLine();
        if (useDynamicHood) {
            telemetry.addData("Target Trajectory", targetTrajectory);
            telemetry.addLine("Target " + targetTrajectory.trajectoryType.name());
            telemetry.addData("Compensated Trajectory", compensatedTrajectory);
            telemetry.addLine("Compensated " + compensatedTrajectory.trajectoryType.name());
        }
        else
            telemetry.addData("Chosen Trajectory", targetTrajectory);
        telemetry.addLine();
        if (controlType == ControlType.EXIT_SPEED) {
            telemetry.addData("ShooterSpeedInRange", trajectoryDistanceLUT.exitSpeedInRange(metersFromGoal, exitSpeedMps, targetTrajectory.exitAngleRad));
            if (useDynamicHood)
                telemetry.addData("CompensatedSpeedInRange", trajectoryDistanceLUT.exitSpeedInRange(metersFromGoal, curExitSpeedMps, targetTrajectory.exitAngleRad));
        }
        else if (controlType == ControlType.EXIT_ANGLE)
            telemetry.addData("ExitAngleInRange", trajectoryDistanceLUT.exitAngleInRange(metersFromGoal, Math.toRadians(exitAngleDeg)));
        else if (controlType == ControlType.OPTIMAL && useDynamicHood)
            telemetry.addData("CompensatedSpeedInRange", trajectoryDistanceLUT.exitSpeedInRange(metersFromGoal, curExitSpeedMps, targetTrajectory.exitAngleRad));

        double targetShooterSpeedTps = ShooterV2.params.getTpsFunction.apply(targetTrajectory.exitSpeedMps);
        double shooterSpeedActualMps = ShooterV2.params.getMpsFunction.apply(shooter.getVelTps());
        boolean inSpeedMOE = Math.abs(shooterSpeedActualMps - targetTrajectory.exitSpeedMps) <= targetTrajectory.exitSpeedMOE;
        telemetry.addLine();
        telemetry.addData("shooterSpeedActualTps", shooter.getVelTps());
        telemetry.addData("shooterSpeedTargetTps", targetShooterSpeedTps);
        telemetry.addData("shooterSpeedMOETps", ShooterV2.params.getTpsFunction.apply(targetTrajectory.exitSpeedMOE));
        telemetry.addData("shooterSpeedActualMps", shooterSpeedActualMps);
        telemetry.addData("shooterSpeedTargetMps", targetTrajectory.exitSpeedMps);
        telemetry.addData("shooterSpeedMOEMps", targetTrajectory.exitSpeedMOE);
        telemetry.addData("shooterSpeedInMOE", inSpeedMOE);
        if (setShooterHoodToTrajectory) {
            shooter.setShooterVelocityPID(targetShooterSpeedTps, batteryVoltageFilter.getVoltage());
            if (useDynamicHood) {
                double[] info = TrajectoryMath.calculateFilteredExitAngle(targetTrajectory, compensatedTrajectory);
                hood.setTargetExitAngle(info[0]);
                telemetry.addData("exitAngleTValue", info[1]);
            }
            else
                hood.setTargetExitAngle(targetTrajectory.exitAngleRad);

            ArrayList<Vector3d> points = targetTrajectory.simulateTrajectory(300, 10, 0, new Vector3d(0, 0, 0), new Vector3d(0, 0, 0));
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

    private TrajectoryWrapper chooseTrajectory(ControlType controlType) {
        return switch (controlType) {
            case EXIT_SPEED ->
                    trajectoryDistanceLUT.getInterpolatedExitSpeedTrajectory(metersFromGoal, exitSpeedMps, Math.toRadians(exitAngleDeg));
            case EXIT_ANGLE ->
                    trajectoryDistanceLUT.getInterpolatedExitAngleTrajectory(metersFromGoal, Math.toRadians(exitAngleDeg));
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
