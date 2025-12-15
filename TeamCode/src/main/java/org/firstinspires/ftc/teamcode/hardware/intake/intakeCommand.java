package org.firstinspires.ftc.teamcode.hardware.intake;

import com.qualcomm.robotcore.util.ElapsedTime;
import dev.nextftc.core.commands.Command;
import org.firstinspires.ftc.teamcode.hardware.intake.IntakeSubsystem;

public class intakeCommand extends Command {
    private final IntakeSubsystem intake;
    private final double durationSeconds;
    private final ElapsedTime timer;

    public intakeCommand(IntakeSubsystem intake, double durationSeconds) {
        this.intake = intake;
        this.durationSeconds = durationSeconds;
        this.timer = new ElapsedTime();

        // Register the subsystem dependency
        addRequirements(intake);
    }

    // RENAMED: initialize() -> onStart()
    @Override
    public void start() {
        timer.reset();
        intake.runIntake(0.8);
    }

    // RENAMED: execute() -> onExecute()
    @Override
    public void update() {
        // Optional: telemetry can go here
    }

    // This matches the abstract method requirement you found earlier
    @Override
    public boolean isDone() {
        return timer.seconds() >= durationSeconds;
    }

    // RENAMED: end(boolean) -> onEnd(boolean)
    @Override
    public void stop(boolean interrupted) {
        intake.stop();
    }
}