package org.firstinspires.ftc.teamcode.auto.pedro;

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
public class RobotRowToRow extends OpMode
{
    public Follower follower; // Pedro Pathing follower instance
    private Paths paths; // Paths defined in the Paths class
    private int pathState; // Current autonomous path state (state machine)

    private IntakeSubsystem intake;
    private CatapultSubsystem catapult;

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance

    private ElapsedTime timer;

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
                catapult.setPower(CatapultSubsystem.POWER_HOLD); // prevents the catapult from snapping up
                CommandManager.INSTANCE.scheduleCommand((new intakeCommand(intake, 5)));
            };

            Runnable shootPhase = () -> {
                CommandManager.INSTANCE.scheduleCommand(new CatapultFireCommand(catapult, telemetry));
            };

            // Each row represents robot moving to row of 3 artifacts and intaking them
            row0 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(24.000, 120.000),
                                    new Pose(84.000, 95.851),
                                    new Pose(22.000, 80)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .addParametricCallback(0.01, shootPhase)
                    .addParametricCallback(0.02, intakePhase)
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
                    .addParametricCallback(0.01, intakePhase)
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
                    .addParametricCallback(0.01, intakePhase)
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
    public int autonomousPathUpdate()
    {
        switch (pathState)
        {
            case 0:
                follower.followPath(paths.row0, 0.7, true);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy())
                {
                    follower.followPath(paths.rowShoot0, 0.7, true);
                    pathState = 2;
                }
                break;
            case 2:
                if (!follower.isBusy())
                {
                    follower.followPath(paths.row1, 0.7, true);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy())
                {
                    follower.followPath(paths.rowShoot1, 0.7, true);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy())
                {
                    follower.followPath(paths.row2, 0.7, true);
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy())
                {
                    follower.followPath(paths.rowShoot2, 0.7, true);
                    pathState = 6;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    panelsTelemetry.update();
                }
                break;
        }

        return pathState;
    }
}