/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.teleop.TeleOpControlLinearOpMode;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: Shooty", group="Robot")
@Disabled
public class RobotAutoDriveShoot extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         leftFront   = null;
    private DcMotor         leftBack   = null;
    private DcMotor         rightFront  = null;
    private DcMotor         rightBack  = null;
    private DcMotor         catapult1 = null;
    private DcMotor         catapult2 = null;

    private ElapsedTime     runtime = new ElapsedTime();
    private enum CatapultModes {UP, DOWN, HOLD}
    private double CATAPULT_UP_POWER = -1.0;
    private double CATAPULT_DOWN_POWER = 1.0;  // Need full power with 12 rubber bands. Half that amount can be adjusted to use 0.5 power.
    private double CATAPULT_HOLD_POWER = 0;
    private CatapultModes pivotMode;
    private static ElapsedTime pivotUpTime = new ElapsedTime();
    private static ElapsedTime pivotDownTime = new ElapsedTime();




    static final double     FORWARD_SPEED = 0.6;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        rightFront  = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBack = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftBack = hardwareMap.get(DcMotor.class, "left_back_drive");
        leftFront = hardwareMap.get(DcMotor.class, "left_front_drive");
        catapult1 = hardwareMap.get(DcMotor.class, "catapult1");
        catapult2 = hardwareMap.get(DcMotor.class, "catapult2");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        catapult1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapult1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        catapult2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        catapult2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        catapult1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult1.setDirection(DcMotor.Direction.REVERSE); // Backwards should pivot DOWN, or in the stowed position.
        catapult2.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way.

        // Step 1:  Drive forward for 3 seconds

            pivotMode = RobotAutoDriveShoot.CatapultModes.UP;
            catapult1.setPower(CATAPULT_UP_POWER);
            catapult2.setPower(CATAPULT_UP_POWER);
            pivotUpTime.reset();
         if (pivotMode == RobotAutoDriveShoot.CatapultModes.UP && pivotUpTime.time() > 0.5) {
            pivotMode = RobotAutoDriveShoot.CatapultModes.DOWN;
            catapult1.setPower(CATAPULT_DOWN_POWER);
            catapult2.setPower(CATAPULT_DOWN_POWER);
            pivotDownTime.reset();
        } else if (pivotMode == RobotAutoDriveShoot.CatapultModes.DOWN && pivotDownTime.time() > 0.1) {
            pivotMode = RobotAutoDriveShoot.CatapultModes.HOLD;
            catapult1.setPower(CATAPULT_HOLD_POWER);
            catapult2.setPower(CATAPULT_HOLD_POWER);
        }





        // Step 4:  Stop

    }
}
