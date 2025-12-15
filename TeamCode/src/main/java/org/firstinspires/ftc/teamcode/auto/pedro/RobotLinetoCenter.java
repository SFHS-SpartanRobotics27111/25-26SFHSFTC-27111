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

@Autonomous(name = "Pedro Pathing Line to Center", group = "Autonomous")
@Configurable // Panels
public class RobotLinetoCenter extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private IntakeSubsystem intake;
    private CatapultSubsystem catapult;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));
        follower.activateAllPIDFs();
        intake = new IntakeSubsystem(hardwareMap);
        catapult = new CatapultSubsystem(hardwareMap);

        paths = new Paths(follower, intake, catapult, telemetry); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        CommandManager.INSTANCE.cancelAll();
    }

    @Override
    public void loop() {
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

        public PathChain Path1;
        public PathChain Path2;
        public PathChain curve;
        public PathChain drive1;
        public PathChain intake1;
        public PathChain gotodepot1;


        public Paths(Follower follower, IntakeSubsystem intake, CatapultSubsystem catapult, Telemetry telemetry) {
            drive1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72, 8), new Pose(72, 72))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(138))
                    .addParametricCallback(0.01, () -> {
                        catapult.setPower(CatapultSubsystem.POWER_HOLD); // prevents the catapult from snapping up
                        CommandManager.INSTANCE.scheduleCommand((new intakeCommand((intake), 2)));

                    })


                    .build();

            intake1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(38, 75), new Pose(7, 75))
                    )
                    .addParametricCallback(0.5, () -> {
                        CommandManager.INSTANCE.scheduleCommand(new intakeCommand(intake, 5.0));
                    })


                    .setTangentHeadingInterpolation()
                    .build();

            gotodepot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72, 72), new Pose(23, 124))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(138))
                    .addParametricCallback(0.99, () -> {
                        CommandManager.INSTANCE.scheduleCommand(new CatapultFireCommand(catapult, telemetry));

            })
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        switch (pathState) {
            case 0:



                follower.followPath(paths.drive1, 0.7, true);
                pathState = 1; // this needs to be set otherwise it shutters BAD

                break;
            case 1:
                if (!follower.isBusy()){

                    follower.followPath(paths.gotodepot1);
                    pathState = 2;

                }

                    break;



            case 2:
                if (!follower.isBusy()){
                    panelsTelemetry.update();
                    break;

                }


        }
        return pathState;
    }
}