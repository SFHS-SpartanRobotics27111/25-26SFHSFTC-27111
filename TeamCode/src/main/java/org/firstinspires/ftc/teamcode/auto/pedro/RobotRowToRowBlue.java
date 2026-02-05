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

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;

@Autonomous(name = "Pedro Pathing Row to Row Blue", group = "Autonomous")
@Configurable
public class RobotRowToRowBlue extends OpMode
{
    public float offSetX = 0;
    public float offSetY = 0;

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
        INTAKE_LINE_0,
        INTAKE_0_SHOOT,
        INTAKE_0_WAIT,

        INTAKE_1,
        INTAKE_LINE_1,
        INTAKE_1_SHOOT,
        INTAKE_1_WAIT,

        INTAKE_2,
        INTAKE_LINE_2,
        INTAKE_2_SHOOT,
        INTAKE_2_WAIT,

        GATE,
        OPEN_GATE,

        TELEMETRY
    }
    private AutoState pathState = AutoState.INIT_SHOT; // Current autonomous path state (state machine)

    @Override
    public void init()
    {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(24 + offSetX, 120 + offSetY, Math.toRadians(135)));
        follower.activateAllPIDFs();

        intake = new IntakeSubsystem(hardwareMap);
        catapult = new CatapultSubsystem(hardwareMap);

        paths = new Paths(follower, intake, catapult, telemetry, offSetX, offSetY); // Build paths

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

        panelsTelemetry.debug("offsetX", offSetX);
        panelsTelemetry.debug("offsetY",offSetY);

        panelsTelemetry.update(telemetry);
    }

    public static class Paths
    {
        public PathChain init_shot;
        public PathChain row0;
        public PathChain rowIntake0;
        public PathChain rowShoot0;
        public PathChain row1;
        public PathChain rowIntake1;
        public PathChain rowShoot1;
        public PathChain row2;
        public PathChain rowIntake2;
        public PathChain rowShoot2;
        public PathChain goToGate;
        public PathChain openGate;

        public Paths(Follower follower, IntakeSubsystem intake, CatapultSubsystem catapult, Telemetry telemetry, float offX, float offY)
        {
            // Defining runnables (lambda functions) externally to make things look cleaner
            Runnable intakePhase = () -> {
                CommandManager.INSTANCE.scheduleCommand((new intakeCommand(intake, 3)));
            };

            Runnable shootPhase = () -> {
                CommandManager.INSTANCE.scheduleCommand(new CatapultFireCommand(catapult, telemetry));
            };


            // Each row represents robot moving to row of 3 artifacts and intaking them
            row0 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(24.000 +offX, 120.000 +offY),
                                    new Pose(80 + offX, 80 +offY),
                                    new Pose(45 +offX, 75 + offY)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            rowIntake0 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(35.000 +offX, 75.000 +offY),

                                    new Pose(10 +offX , 75.000 +offY)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPoseCallback(new Pose(35 +offX, 75 +offY), intakePhase, 0.01)
                    .build();

            // Each row shoot represents path robot takes to go shoot the artifacts
            rowShoot0 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(22.000 +offX, 80 +offY),
                                    new Pose(19.036 +offX, 105.928 +offY),
                                    new Pose(22 +offX, 120.000 +offY)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(135))
                    .addParametricCallback(0.97, shootPhase) //always shoots if this value is lower
                    .build();

            row1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(22 +offX, 120.000 +offY),
                                    new Pose(64 +offX, 56 +offY),
                                    new Pose(45 +offX, 53 +offY)

                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();
            rowIntake1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(45 +offX, 53 +offY),

                                    new Pose(5 +offX, 53 +offY)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPoseCallback(new Pose(45 +offX, 53 +offY), intakePhase, 0.01)
                    .build();

            rowShoot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(5 +offX, 53 +offY),
                                    new Pose(59 +offX, 69 +offY),
                                    new Pose(22 +offX, 120.000 +offY)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(135))
                    .addParametricCallback(0.97, shootPhase)
                    .build();

            row2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(22 +offX, 120.000 +offY),
                                    new Pose(64.000 +offX, 30.000 +offY),
                                    new Pose(45.000 +offX, 25.000 +offY)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();
            rowIntake2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(45.000 +offX, 30 +offY),

                                    new Pose(3 +offX, 30 +offY)
                            )
                    ).setTangentHeadingInterpolation()
                    .addPoseCallback(new Pose(45 +offX, 30 +offY), intakePhase, 0.01)
                    .build();

            rowShoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(10 +offX, 25 +offY),
                                    new Pose(31 +offX, 100 +offY),
                                    new Pose(22 +offX, 120.000 +offY)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(135))
                    .addParametricCallback(0.97, shootPhase)
                    .build();
            goToGate = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(22.000 +offX, 120.000 +offY),

                                    new Pose(30.000 +offX, 75.000 +offY)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();
            openGate = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(30.000 +offX, 75.000 +offY),

                                    new Pose(22 +offX, 65 +offY)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
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
                timer.reset();
                CommandManager.INSTANCE.scheduleCommand(new CatapultFireCommand(catapult, telemetry));
                pathState = AutoState.INTAKE_0;
                break;

            // ---------------------------------------------------------------------------

            // First intake and shot
            case INTAKE_0:

                if (timer.time() > 0.1)
                {
                    follower.followPath(paths.row0, 0.9, true);

                    pathState = AutoState.INTAKE_LINE_0;
                }
                break;

            case INTAKE_LINE_0:
                if (!follower.isBusy())
                {
                    follower.followPath(paths.rowIntake0, 0.5, true);
                    pathState = AutoState.INTAKE_0_SHOOT;
                }

                break;

            case INTAKE_0_SHOOT:
                if (!follower.isBusy())
                {
                    follower.followPath(paths.rowShoot0, 0.9, true);
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
                if (timer.time() > 0.2)
                {
                    follower.followPath(paths.row1, 0.9, true);
                    pathState = AutoState.INTAKE_LINE_1;
                }
                break;
            case INTAKE_LINE_1:
                if (!follower.isBusy())
                {
                    follower.followPath(paths.rowIntake1, 0.5, true);
                    pathState = AutoState.INTAKE_1_SHOOT;
                }

                break;

            case INTAKE_1_SHOOT:
                if (!follower.isBusy())
                {
                    follower.followPath(paths.rowShoot1, 0.9, true);
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
                if (timer.time() > 0.2)
                {
                    follower.followPath(paths.row2, 0.9, true);
                    pathState = AutoState.INTAKE_LINE_2;
                }
                break;
            case INTAKE_LINE_2:
                if (!follower.isBusy())
                {
                    follower.followPath(paths.rowIntake2, 0.5, true);
                    pathState = AutoState.INTAKE_2_SHOOT;
                }

                break;

            case INTAKE_2_SHOOT:
                if (!follower.isBusy())
                {
                    follower.followPath(paths.rowShoot2, 0.9, true);
                    timer.reset();
                    pathState = AutoState.INTAKE_2_WAIT;
                }
                break;

            case INTAKE_2_WAIT:
                if (!follower.isBusy())
                {
                    timer.reset();
                    pathState = AutoState.GATE;
                }
                break;

            case GATE:
                if (!follower.isBusy())
                {
                    follower.followPath(paths.goToGate, 0.9, true);
                    pathState = AutoState.OPEN_GATE;
                }
                break;
            case OPEN_GATE:
                if (!follower.isBusy())
                {
                    follower.followPath(paths.openGate, 0.9, true);
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