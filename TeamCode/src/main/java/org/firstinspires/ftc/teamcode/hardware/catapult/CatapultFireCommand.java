package org.firstinspires.ftc.teamcode.hardware.catapult;

import com.qualcomm.robotcore.util.ElapsedTime;
import dev.nextftc.core.commands.Command;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CatapultFireCommand extends Command {

    private final CatapultSubsystem catapult;
    private final ElapsedTime timer;
    private Stage stage;
    private final Telemetry telemetry; // For debugging

    private enum Stage {
        FIRING,
        RESETTING,
        COMPLETE
    }

    // Constructor now accepts Telemetry
    public CatapultFireCommand(CatapultSubsystem catapult, Telemetry telemetry) {
        this.catapult = catapult;
        this.telemetry = telemetry;
        this.timer = new ElapsedTime();
        addRequirements(catapult);
    }

    @Override
    public void start() {
        timer.reset();
        stage = Stage.FIRING;
        catapult.setPower(CatapultSubsystem.POWER_UP); // Should be -1.0
    }

    @Override
    public void update() {
        // DEBUGGING: Print status to the phone

        if (telemetry != null) {
            telemetry.addData("Catapult Stage", stage);
            telemetry.addData("Catapult Timer", timer.seconds());
            telemetry.update();
        }

        // Logic
        if (stage == Stage.FIRING && timer.seconds() > 0.5) {
            stage = Stage.RESETTING;
            catapult.setPower(CatapultSubsystem.POWER_DOWN); // Should be 1.0
            timer.reset();
        }
        else if (stage == Stage.RESETTING && timer.seconds() > 0.1) {
            stage = Stage.COMPLETE;
        }
    }

    @Override
    public boolean isDone() {
        return stage == Stage.COMPLETE;
    }

    // Using 'stop' as verified by your environment
    @Override
    public void stop(boolean interrupted) {
        catapult.setPower(CatapultSubsystem.POWER_HOLD); // Should be 0.2
        if (telemetry != null) {
            telemetry.addData("Catapult Status", "FINISHED / HOLDING");
            telemetry.update();
        }
    }
}