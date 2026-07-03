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
import org.firstinspires.ftc.teamcode.utils.offboardShooting.math.Angle2d;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories.TrajectoryDistanceLUT;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.trajectories.TrajectoryLoader;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.math.TrajectoryMath;
import org.firstinspires.ftc.teamcode.utils.offboardShooting.math.Vec3d;

import java.text.DecimalFormat;
import java.util.ArrayList;

@TeleOp(name="TrajectoryDistanceLUTTest", group="OffboardShooting")
@Config
public class TrajectoryDistanceLUTTest extends OpMode {
    private static final DecimalFormat df = new DecimalFormat("0.000");

    public enum ControlType {
        EXIT_SPEED,
        EXIT_ANGLE,
        OPTIMAL
    }
    public static class HardwareControls {
        public boolean setShooterHoodToTrajectory = false;
        public boolean runIntake = false;
        public boolean engageClutch = false;
    }
    public static class ShootingControls {
        public double metersFromGoal = 3;
        public boolean useDynamicHood = false;
        public ControlType controlType = ControlType.EXIT_SPEED;
        public double[] robotLinearVel = new double[] { 0, 0 };
        public Angle2d robotAngularVel = Angle2d.kZero;
        public int numTofIterations = 5;
    }
    public static class ExitAngleControls {
        public double exitAngleDeg = 0;
    }
    public static class ExitSpeedControls {
        public double exitSpeedMps = 0;
        public boolean highArc = true;
    }
    public static class OptimalControls {
        public boolean highArc = true;
    }
    public static HardwareControls hardwareControls = new HardwareControls();
    public static ShootingControls shootingControls = new ShootingControls();
    public static ExitAngleControls exitAngleControls = new ExitAngleControls();
    public static ExitSpeedControls exitSpeedControls = new ExitSpeedControls();
    public static OptimalControls optimalControls = new OptimalControls();
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
        hardwareControls.setShooterHoodToTrajectory = false;
        hardwareControls.runIntake = false;
        hardwareControls.engageClutch = false;
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
        addNeighborLUTTelemetry();
        telemetry.update();
    }

    @Override
    public void loop() {
        updateDriver1Controls();
        updateCollectorState();

        telemetry.addData("SetShooterHoodToTrajectory", hardwareControls.setShooterHoodToTrajectory);
        telemetry.addData("RunIntake", hardwareControls.runIntake);
        telemetry.addData("EngageClutch", hardwareControls.engageClutch);
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

        Trajectory targetTrajectory = chooseTrajectory(shootingControls.controlType);
        Trajectory compensatedTrajectory = trajectoryDistanceLUT.getInterpolatedExitSpeedTrajectory(shootingControls.metersFromGoal, curExitSpeedMps, targetTrajectory.exitAngle);
        Trajectory commandedTrajectory = shootingControls.useDynamicHood ? compensatedTrajectory : targetTrajectory;

        telemetry.addData("Control Type", shootingControls.controlType.name());
        telemetry.addLine("Aiming " + shootingControls.metersFromGoal + " meters forward from exit position");
        telemetry.addLine("Aiming " + trajectoryDistanceLUT.getRelGoalHeight() + " meters up from exit position");
        telemetry.addData("DistanceInRange", trajectoryDistanceLUT.distanceInRange(shootingControls.metersFromGoal));
        telemetry.addLine();
        telemetry.addData("ShooterSpeedRange", df.format(trajectoryDistanceLUT.getMinExitSpeedMps(shootingControls.metersFromGoal)) + "-" + df.format(trajectoryDistanceLUT.getMaxExitSpeedMps(shootingControls.metersFromGoal, targetTrajectory.exitAngle)));
        telemetry.addData("ExitAngleRange", df.format(trajectoryDistanceLUT.getMinExitAngle(shootingControls.metersFromGoal).degrees()) + "-" + df.format(trajectoryDistanceLUT.getMaxExitAngle(shootingControls.metersFromGoal).degrees()));
        telemetry.addLine();
        if (shootingControls.useDynamicHood) {
            telemetry.addData("Target Trajectory", targetTrajectory);
            telemetry.addData("Target OnTarget", targetTrajectory.onTarget);
            telemetry.addData("Compensated Trajectory", compensatedTrajectory);
            telemetry.addData("Compensated OnTarget", compensatedTrajectory.onTarget);

        }
        else
            telemetry.addData("Chosen Trajectory", targetTrajectory);
        telemetry.addLine();
        if (shootingControls.controlType == ControlType.EXIT_SPEED) {
            telemetry.addData("ShooterSpeedInRange", trajectoryDistanceLUT.exitSpeedInRange(shootingControls.metersFromGoal, exitSpeedControls.exitSpeedMps, targetTrajectory.exitAngle));
            if (shootingControls.useDynamicHood)
                telemetry.addData("CompensatedSpeedInRange", trajectoryDistanceLUT.exitSpeedInRange(shootingControls.metersFromGoal, curExitSpeedMps, targetTrajectory.exitAngle));
        }
        else if (shootingControls.controlType == ControlType.EXIT_ANGLE)
            telemetry.addData("ExitAngleInRange", trajectoryDistanceLUT.exitAngleInRange(shootingControls.metersFromGoal, Angle2d.fromDegrees(exitAngleControls.exitAngleDeg)));
        else if (shootingControls.controlType == ControlType.OPTIMAL && shootingControls.useDynamicHood)
            telemetry.addData("CompensatedSpeedInRange", trajectoryDistanceLUT.exitSpeedInRange(shootingControls.metersFromGoal, curExitSpeedMps, targetTrajectory.exitAngle));

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
        if (hardwareControls.setShooterHoodToTrajectory) {
            shooter.setShooterVelocityPID(targetShooterSpeedTps, batteryVoltageFilter.getVoltage());
            if (shootingControls.useDynamicHood) {
                double[] info = TrajectoryMath.calculateFilteredExitAngle(targetTrajectory, compensatedTrajectory);
                hood.setTargetExitAngle(info[0]);
                telemetry.addData("exitAngleTValue", info[1]);
            }
            else
                hood.setTargetExitAngle(targetTrajectory.exitAngle.radians());

            ArrayList<Vec3d> points = commandedTrajectory.simulateTrajectory(300, 10, Angle2d.fromRadians(0), new Vec3d(0, 0, 0), new Vec3d(0, 0, 0));
            Vec3d startingPositionMeters = points.get(0);
            Vec3d startingPositionFeet = startingPositionMeters.times(3.281);
            Vec3d landingPositionMeters = points.get(points.size() - 1);
            Vec3d landingPositionFeet = landingPositionMeters.times(3.281);

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setFill("green");
            packet.fieldOverlay().fillCircle(startingPositionFeet.x(), startingPositionFeet.y(), 1);
            packet.fieldOverlay().setFill("red");
            packet.fieldOverlay().fillCircle(landingPositionFeet.x(), landingPositionFeet.y(), 1);

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
        if (!hardwareControls.setShooterHoodToTrajectory) {
            shooter.stopMotor();
        }

        telemetry.update();
    }

    private void reloadTrajectoryDistanceLUT() {
        trajectoryDistanceLUT = TrajectoryLoader.loadTrajectoryDistanceLUT("mtiTrajectories.json");
    }

    private void addNeighborLUTTelemetry() {
        if (!trajectoryDistanceLUT.distanceInRange(shootingControls.metersFromGoal)) {
            telemetry.addData("shootingControls.metersFromGoal out of LUT range", shootingControls.metersFromGoal);
            telemetry.addData("range", df.format(trajectoryDistanceLUT.getMinDistance()) + "-" + df.format(trajectoryDistanceLUT.getMaxDistance()));
            return;
        }

        TrajectoryDistanceLUT.NeighborTrajectoryInfo info = trajectoryDistanceLUT.getNeighboringTrajectoryLUTs(shootingControls.metersFromGoal);
        telemetry.addData("loLUT", info.loLUT());
        telemetry.addData("hiLUT", info.hiLUT());
        telemetry.addData("loLUT speed sorted", info.loLUT().getSpeedSortedTrajectoriesString());
        telemetry.addData("hiLUT speed sorted", info.hiLUT().getSpeedSortedTrajectoriesString());
    }

    private Trajectory chooseTrajectory(ControlType controlType) {
        return switch (controlType) {
            case EXIT_SPEED ->
                    trajectoryDistanceLUT.getInterpolatedExitSpeedTrajectory(
                            shootingControls.metersFromGoal,
                            exitSpeedControls.exitSpeedMps,
                            exitSpeedControls.highArc
                    );
            case EXIT_ANGLE ->
                    trajectoryDistanceLUT.getInterpolatedExitAngleTrajectory(
                            shootingControls.metersFromGoal,
                            Angle2d.fromDegrees(exitAngleControls.exitAngleDeg)
                    );
            case OPTIMAL ->
                    trajectoryDistanceLUT.getInterpolatedOptimalTrajectory(
                            shootingControls.metersFromGoal,
                            optimalControls.highArc
                    );
        };
    }

    private void updateDriver1Controls() {
        if (gamepad1.yWasPressed())
            hardwareControls.setShooterHoodToTrajectory = !hardwareControls.setShooterHoodToTrajectory;
        if (gamepad1.bWasPressed()) {
            hardwareControls.engageClutch = !hardwareControls.engageClutch;
            if (hardwareControls.engageClutch)
                hardwareControls.runIntake = false;
        }
        if (!hardwareControls.engageClutch)
            hardwareControls.runIntake = gamepad1.right_trigger > 0.3;
        else if (gamepad1.aWasPressed())
            hardwareControls.runIntake = !hardwareControls.runIntake;
    }

    private void updateCollectorState() {
        if (hardwareControls.runIntake)
            collector.setIntakeState(Collector.IntakeState.INTAKE);
        else
            collector.setIntakeState(Collector.IntakeState.OFF);

        if (hardwareControls.engageClutch)
            collector.setClutchState(Collector.ClutchState.ENGAGED);
        else
            collector.setClutchState(Collector.ClutchState.DISENGAGED);
    }
}
