package org.firstinspires.ftc.teamcode.auto.pedro;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.catapult.CatapultFireCommand;
import org.firstinspires.ftc.teamcode.hardware.catapult.CatapultSubsystem;
import org.firstinspires.ftc.teamcode.hardware.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.hardware.intake.intakeCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.core.commands.CommandManager;

@Autonomous(name = "Pedro Pathing Row to Row", group = "Autonomous")
@Configurable
public class RobotRowToRowBlue extends OpMode
{
    public Follower follower; // Pedro Pathing follower instance
    private Paths paths; // Paths defined in the Paths class

    private IntakeSubsystem intake;
    private CatapultSubsystem catapult;

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance

    private ElapsedTime timer = new ElapsedTime();

    public enum AutoState
    {
        INIT_SHOT,
        INIT_SHOT_WAIT,

        INTAKE_0,
        INTAKE_0_SHOOT,
        INTAKE_0_WAIT,

        INTAKE_1,
        INTAKE_1_SHOOT,
        INTAKE_1_WAIT,

        INTAKE_2,
        INTAKE_2_SHOOT,

        TELEMETRY
    }
    private AutoState pathState = AutoState.INIT_SHOT; // Current autonomous path state (state machine)

    @Override
    public void init()
    {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(24, 120, Math.toRadians(135)));
        follower.activateAllPIDFs();

        intake = new IntakeSubsystem(hardwareMap);
        catapult = new CatapultSubsystem(hardwareMap);

        paths = new Paths(follower, intake, catapult, telemetry); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        CommandManager.INSTANCE.cancelAll();
    }

    @Override
    public void loop()
    {
        follower.update(); // Update Pedro Pathing

        pathState = autonomousPathUpdate(); // Update autonomous state machine

        CommandManager.INSTANCE.run();

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());

        panelsTelemetry.update(telemetry);
    }

    public static class Paths
    {
        public PathChain init_shot;
        public PathChain row0;
        public PathChain rowShoot0;
        public PathChain row1;
        public PathChain rowShoot1;
        public PathChain row2;
        public PathChain rowShoot2;

        public Paths(Follower follower, IntakeSubsystem intake, CatapultSubsystem catapult, Telemetry telemetry)
        {
            // Defining runnables (lambda functions) externally to make things look cleaner
            Runnable intakePhase = () -> {
                CommandManager.INSTANCE.scheduleCommand((new intakeCommand(intake, 5)));
            };

            Runnable shootPhase = () -> {
                CommandManager.INSTANCE.scheduleCommand(new CatapultFireCommand(catapult, telemetry));
            };

            // Empty path for just shooting
            init_shot = follower
                    .pathBuilder()
                    .addParametricCallback(0.01, () -> {
                        catapult.setPower(CatapultSubsystem.POWER_HOLD); // prevents the catapult from snapping up
                    })
                    .addParametricCallback(0.99, shootPhase)
                    .build();

            // Each row represents robot moving to row of 3 artifacts and intaking them
            row0 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(24.000, 120.000),
                                    new Pose(80, 80),
                                    new Pose(22.000, 74)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .addParametricCallback(0.5, intakePhase)
                    .build();

            // Each row shoot represents path robot takes to go shoot the artifacts
            rowShoot0 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(22.000, 80),
                                    new Pose(19.036, 105.928),
                                    new Pose(24.000, 120.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(135))
                    .addParametricCallback(0.99, shootPhase)
                    .build();

            row1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(24.000, 120.000),
                                    new Pose(80.000, 56.000),
                                    new Pose(22.000, 56)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .addParametricCallback(0.6, intakePhase)
                    .build();

            rowShoot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(22.000, 56),
                                    new Pose(17.916, 103.241),
                                    new Pose(24.000, 120.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(135))
                    .addParametricCallback(0.99, shootPhase)
                    .build();

            row2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(24.000, 120.000),
                                    new Pose(80.000, 30.000),
                                    new Pose(22.000, 32)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .addParametricCallback(0.7, intakePhase)
                    .build();

            rowShoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(22.000, 32),
                                    new Pose(18.140, 108.840),
                                    new Pose(24.000, 120.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(135))
                    .addParametricCallback(0.99, shootPhase)
                    .build();
        }
    }

    // Pathing state machine
    // Manages each part of the auto routine
    public AutoState autonomousPathUpdate()
    {
        switch (pathState)
        {
            // ---------------------------------------------------------------------------

            // Initial shot with pre-loaded artifacts
            case INIT_SHOT:
                follower.followPath(paths.init_shot, 0.7, true);
                pathState = AutoState.INIT_SHOT_WAIT;
                break;
            case INIT_SHOT_WAIT:
                if (!follower.isBusy())
                {
                    timer.reset();
                    pathState = AutoState.INTAKE_0;
                }
                break;

            // ---------------------------------------------------------------------------

            // First intake and shot
            case INTAKE_0:
                if (timer.time() > 1)
                {
                    follower.followPath(paths.row0, 0.7, true);
                    pathState = AutoState.INTAKE_0_SHOOT;
                }
                break;

            case INTAKE_0_SHOOT:
                if (!follower.isBusy())
                {
                    follower.followPath(paths.rowShoot0, 0.7, true);
                    timer.reset();
                    pathState = AutoState.INTAKE_0_WAIT;
                }
                break;
            case INTAKE_0_WAIT:
                if (!follower.isBusy())
                {
                    timer.reset();
                    pathState = AutoState.INTAKE_1;
                }
                break;

            // ---------------------------------------------------------------------------

            // Second intake and shot
            case INTAKE_1:
                if (timer.time() > 1)
                {
                    follower.followPath(paths.row1, 0.7, true);
                    pathState = AutoState.INTAKE_1_SHOOT;
                }
                break;

            case INTAKE_1_SHOOT:
                if (!follower.isBusy())
                {
                    follower.followPath(paths.rowShoot1, 0.7, true);
                    timer.reset();
                    pathState = AutoState.INTAKE_1_WAIT;
                }
                break;
            case INTAKE_1_WAIT:
                if (!follower.isBusy())
                {
                    timer.reset();
                    pathState = AutoState.INTAKE_2;
                }
                break;

            // ---------------------------------------------------------------------------

            // Third intake and shot
            case INTAKE_2:
                if (timer.time() > 1)
                {
                    follower.followPath(paths.row2, 0.7, true);
                    pathState = AutoState.INTAKE_2_SHOOT;
                }
                break;

            case INTAKE_2_SHOOT:
                if (!follower.isBusy())
                {
                    follower.followPath(paths.rowShoot2, 0.7, true);
                    timer.reset();
                    pathState = AutoState.TELEMETRY;
                }
                break;

            // ---------------------------------------------------------------------------

            // The end
            case TELEMETRY:
                if (!follower.isBusy()) {
                    panelsTelemetry.update();
                }
                break;
        }

        return pathState;
    }
}