package org.firstinspires.ftc.teamcode.hardware.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.core.subsystems.Subsystem;

public class IntakeSubsystem implements Subsystem {
    private final DcMotorEx intakeMotor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        // "intake" must match your Driver Station configuration
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        // Optional: Configure motor direction or behavior
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void runIntake(double power) {
        intakeMotor.setPower(power);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }
}