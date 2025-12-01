package org.firstinspires.ftc.teamcode.teleop; //teamcode package is necessary for code to function with out it you are missing hella classes and libs

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import all of our hardware classes. this makes adding new things easier without borking anything else

import org.firstinspires.ftc.teamcode.hardware.OmniDrive;

//TeleOp Parameter is needed to tell the control hub that it should show up in the TeleOp menu and not the Auto menu
@TeleOp(name = "Field Centric Omni TeleOp", group = "Robot")
@Disabled
public class RobotTeleopPOV_Linear extends LinearOpMode {

    @Override
    public void runOpMode() {

        // set up motors
        OmniDrive drive = new OmniDrive(this);

        // since the ending of auto will have a different orientation per which color you started in
        // reset the yaw before doing anything else to avoid directions being cooked
        drive.imu.resetYaw();

        telemetry.addData(">", "Setup complete. Press start");
        telemetry.update();

        waitForStart();

        //main event loop. while the op mode is active we can take in the following inputs and turn it actions on the Bot
        while (opModeIsActive()) {
            // joystick y is negative for forward, so negate it
            // labels are for Playstation controller but the library also has labels for Xbox controllers if you go back to it.
            drive.driveFirstPerson(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.options);
            telemetry.update();
        }
    }
}