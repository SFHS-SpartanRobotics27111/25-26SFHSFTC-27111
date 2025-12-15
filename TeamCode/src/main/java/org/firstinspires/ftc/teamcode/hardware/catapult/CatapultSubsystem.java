package org.firstinspires.ftc.teamcode.hardware.catapult;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.core.subsystems.Subsystem;

public class CatapultSubsystem implements Subsystem {

    // Hardware
    private final DcMotorEx catapult1;
    private final DcMotorEx catapult2;

    // Tuning Constants (Public so they can be accessed if needed)
    public static final double POWER_UP = -1.0;
    public static final double POWER_DOWN = 1.0;
    public static final double POWER_HOLD = 0.2;

    public CatapultSubsystem(HardwareMap hardwareMap) {
        catapult1 = hardwareMap.get(DcMotorEx.class, "catapult1");
        catapult2 = hardwareMap.get(DcMotorEx.class, "catapult2");

        // Configure Motors (Directly from your TeleOp logic)
        catapult1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        catapult1.setDirection(DcMotor.Direction.REVERSE);
        catapult2.setDirection(DcMotor.Direction.FORWARD);

        catapult1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapult2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        catapult1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        catapult2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Helper method to set both motors at once
    public void setPower(double power) {
        catapult1.setPower(power);
        catapult2.setPower(power);
    }

    // Safety method
    public void stop() {
        setPower(0);
    }
}