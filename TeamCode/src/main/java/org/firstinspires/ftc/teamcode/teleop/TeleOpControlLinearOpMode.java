/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.OmniDrive;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

// Based on the sample: Basic: Omni Linear OpMode
@TeleOp(name = "TeleOp Field Centric", group = "Teleop")
public class TeleOpControlLinearOpMode extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime catatime = new ElapsedTime();

    // Declare drive motors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // Declare end-effector members
    private DcMotor intake = null;
    private DcMotor catapult1 = null;
    private DcMotor catapult2 = null;
    private DcMotor foot = null;

    // motor power 1 = 100% and 0.5 = 50%
    // negative values = reverse ex: -0.5 = reverse 50%
    private double INTAKE_IN_POWER = 0.6;
    private double INTAKE_OUT_POWER = -0.9;
    private double INTAKE_OFF_POWER = 0.0;
    private double intakePower = INTAKE_OFF_POWER;

    private double FOOT_UP_POWER = 0.2;
    private double FOOT_DOWN_POWER = -0.2;
    private double FOOT_OFF_POWER = 0.0;
    private double footPower = FOOT_OFF_POWER;

    private double CATAPULT_UP_POWER = -1.0;
    private double CATAPULT_DOWN_POWER = 1.0;  // Need full power with 12 rubber bands. Half that amount can be adjusted to use 0.5 power.
    private double CATAPULT_HOLD_POWER = 0; // Only use a small amount of power to hold it down once it is down, othewise the motor will get very hot from stalling and can damage itself

    private enum CatapultModes {UP, DOWN, HOLD}

    private CatapultModes pivotMode;

    private static ElapsedTime pivotUpTime = new ElapsedTime();
    private static ElapsedTime pivotDownTime = new ElapsedTime();

    private enum FootMode {UP, DOWN, BRAKE}

    private FootMode footmode;

    /*
     * Code to run ONCE when the driver hits INIT (same as previous year's init())
     */
    @Override
    public void runOpMode()
    {
        awake(); // initialize all the variables
        initialize(); // any thing else that needs to be started
        update(); // run until the end of the match (driver presses STOP)
    }

    private void awake()
    {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        intake = hardwareMap.get(DcMotor.class, "intake");
        catapult1 = hardwareMap.get(DcMotor.class, "catapult1");
        catapult2 = hardwareMap.get(DcMotor.class, "catapult2");
        foot = hardwareMap.get(DcMotor.class, "foot");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed
        // to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.

        // reset encoders and set runmode of drive motors
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set direction of wheel motors (IMPORTANT)
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // set wheel motors to BRAKE mode for easier control
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // reset encoders and set runmode of drive motors
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        catapult1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapult1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        catapult2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapult2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        foot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        foot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set direction of subsystem motors
        intake.setDirection(DcMotor.Direction.FORWARD); // Forward should INTAKE.
        catapult1.setDirection(DcMotor.Direction.REVERSE); // Backwards should pivot DOWN, or in the stowed position.
        catapult2.setDirection(DcMotor.Direction.FORWARD);
        foot.setDirection(DcMotor.Direction.REVERSE); // Backwards should should stay UP, or in the stowed position

        // set subsystem motors to BRAKE mode
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        foot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initialize()
    {
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        catatime.reset();
    }

    private void update()
    {
        while (opModeIsActive())
        {
            boolean intakeInButton = gamepad2.left_trigger > 0.2;
            boolean intakeOutButton = gamepad2.left_bumper;

            // This conditional reduces ambiguity when multiple buttons are pressed.
            if (intakeInButton && intakeOutButton) {
                intakeInButton = false;
            }

            boolean catapultFireButton = gamepad2.right_bumper;

            // DRIVE CODE (IMPORTANT)
            OmniDrive drive = new OmniDrive(this);
            drive.driveFirstPerson(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    gamepad1.options
            );

            // INTAKE CODE
            if (intakeInButton) {
                intakePower = INTAKE_IN_POWER;
            } else if (intakeOutButton) {
                intakePower = INTAKE_OUT_POWER;
            } else {
                intakePower = INTAKE_OFF_POWER;
            }

            boolean footOutButton = gamepad1.circle;
            boolean footUpButton = gamepad1.cross;
            boolean footOffButton = gamepad1.square;

            // FOOT CODE
            if (footOutButton) {
                footmode = FootMode.DOWN;
                footPower = FOOT_DOWN_POWER;
            } else if (footUpButton) {
                footmode = FootMode.UP;
                footPower = FOOT_UP_POWER;
            } else if (footOffButton) {
                footmode = FootMode.BRAKE;
                footPower = FOOT_OFF_POWER;
            }

            // PIVOT MODE
            if (catapultFireButton) {
                pivotMode = CatapultModes.UP;
                catapult1.setPower(CATAPULT_UP_POWER);
                catapult2.setPower(CATAPULT_UP_POWER);
                pivotUpTime.reset();
            } else if (pivotMode == CatapultModes.UP && pivotUpTime.time() > 0.5) {
                pivotMode = CatapultModes.DOWN;
                catapult1.setPower(CATAPULT_DOWN_POWER);
                catapult2.setPower(CATAPULT_DOWN_POWER);
                pivotDownTime.reset();
            } else if (pivotMode == CatapultModes.DOWN && pivotDownTime.time() > 0.4) {
                pivotMode = CatapultModes.HOLD;
                catapult1.setPower(CATAPULT_HOLD_POWER);
                catapult2.setPower(CATAPULT_HOLD_POWER);
            }

            intake.setPower(intakePower);
            foot.setPower(footPower);

            String catapult_mode_str;
            if (pivotMode == CatapultModes.UP) {
                catapult_mode_str = "UP";
            } else if (pivotMode == CatapultModes.DOWN) {
                catapult_mode_str = "DOWN";
            } else {
                catapult_mode_str = "HOLD";
            }

            // UPDATE TELEMETRY

            // Show the elapsed game time, wheel power, and other systems power
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Intake", "%%4.2f", intake.getPower());
            telemetry.addData("Foot Power", "%4.2f", foot.getPower());
            telemetry.addData("Foot MODE", "%s", footmode);

            telemetry.addData("Catapult1 Position/power", "%d, %4.2f",
                    catapult1.getCurrentPosition(), catapult1.getPower());
            telemetry.addData("Catapult2 Position/power", "%d, %4.2f",
                    catapult2.getCurrentPosition(), catapult2.getPower());
            telemetry.addData("Catapult MODE", "%s", catapult_mode_str);

            telemetry.update();
        }
    }
}
