package org.firstinspires.ftc.teamcode.auto.pedro;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.catapult.CatapultFireCommand;
import org.firstinspires.ftc.teamcode.hardware.catapult.CatapultSubsystem;
import org.firstinspires.ftc.teamcode.hardware.intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.hardware.intake.intakeCommand;

import dev.nextftc.core.commands.CommandManager;

@Autonomous(name = "Pedro Pathing Row to Row", group = "Autonomous")
@Configurable // Panels
public class RobotRowToRow extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private IntakeSubsystem intake;
    private CatapultSubsystem catapult;

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

    public static class Paths {

        public PathChain row0;
        public PathChain rowShoot0;
        public PathChain row1;
        public PathChain rowShoot1;
        public PathChain row2;
        public PathChain rowShoot2;

        public Paths(Follower follower, IntakeSubsystem intake, CatapultSubsystem catapult, Telemetry telemetry)
        {
            row0 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(24.000, 120.000),
                                    new Pose(84.000, 95.851),
                                    new Pose(22.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            rowShoot0 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(22.000, 84.000),
                                    new Pose(19.036, 105.928),
                                    new Pose(24.000, 120.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(135))
                    .build();

            row1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(24.000, 120.000),
                                    new Pose(80.000, 56.000),
                                    new Pose(22.000, 60.243)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            rowShoot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(22.000, 60.243),
                                    new Pose(17.916, 103.241),
                                    new Pose(24.000, 120.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(135))
                    .build();

            row2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(24.000, 120.000),
                                    new Pose(80.000, 30.000),
                                    new Pose(22.000, 36.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            rowShoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(22.000, 36.000),
                                    new Pose(18.140, 108.840),
                                    new Pose(24.000, 120.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(135))
                    .build();
        }
    }

    public int autonomousPathUpdate()
    {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        switch (pathState)
        {
            case 0:
                follower.followPath(paths.row0, 0.7, true);
                pathState = 1; // this needs to be set otherwise it shutters BAD
                break;
            case 1:
                if (!follower.isBusy())
                {
                    follower.followPath(paths.rowShoot0, 0.7, true);
                    pathState = 2; // this needs to be set otherwise it shutters BAD
                    break;
                }

            case 2:
                if (!follower.isBusy())
                {
                    follower.followPath(paths.row1, 0.7, true);
                    pathState = 3; // this needs to be set otherwise it shutters BAD
                    break;
                }

            case 3:
                if (!follower.isBusy())
                {
                    follower.followPath(paths.rowShoot1, 0.7, true);
                    pathState = 4; // this needs to be set otherwise it shutters BAD
                    break;
                }
            case 4:
                if (!follower.isBusy())
                {
                    follower.followPath(paths.row2, 0.7, true);
                    pathState = 5; // this needs to be set otherwise it shutters BAD
                    break;
                }
            case 5:
                if (!follower.isBusy())
                {
                    follower.followPath(paths.rowShoot2, 0.7, true);
                    pathState = 6; // this needs to be set otherwise it shutters BAD
                    break;
                }
            case 6:
                if (!follower.isBusy()) {
                    panelsTelemetry.update();
                }
                break;
        }
        return pathState;
    }
}