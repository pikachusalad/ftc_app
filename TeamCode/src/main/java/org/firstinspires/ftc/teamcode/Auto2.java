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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="MK_Auto2", group="Auto2")

public class Auto2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor FR = null;

    private static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // REV Hex HD 40:1
    static final double     WHEEL_DIAMETER_INCHES   = 3.54331;     // For figuring circumference
    static final double     WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV  / WHEEL_CIRCUMFERENCE_INCHES);
    static final double     DRIVE_SPEED             = 0.1;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.addData("circumferance", WHEEL_CIRCUMFERENCE_INCHES);
        telemetry.addData("counts per inch", COUNTS_PER_INCH);
        telemetry.addData("counts per foot", 12 * COUNTS_PER_INCH);
        telemetry.addData("int counts per foot", (int) (12 * COUNTS_PER_INCH));
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);

        // we want the the thing to stop, not keep rolling
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        DriveEncoder(DRIVE_SPEED, 12);
        sleep(3000);
        TurnRight();
        sleep(3000);
        DriveEncoder(DRIVE_SPEED, 12);
        sleep(3000);
        TurnLeft();
        sleep(3000);
        StopDriving();

//        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        FL.setTargetPosition(2240);
//        FL.setPower(0.1);
//
//        while (FL.isBusy()) {
//            telemetry.addData("FL", FL.getCurrentPosition());
//            telemetry.update();
//
//        }
//
//        FL.setPower(0);


    }

    public void DriveEncoder(double speed, double inches) {
        int newLeftTarget;
        int newRightTarget;

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
//            newLeftTarget = FL.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
//            newRightTarget = FR.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newLeftTarget = (int) (inches * COUNTS_PER_INCH);
            newRightTarget = (int) (inches * COUNTS_PER_INCH);
            FL.setTargetPosition(newLeftTarget);
            FR.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FL.setPower(speed);
            FR.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (FL.isBusy() && FR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("right target", newRightTarget);
                telemetry.addData("right current", FR.getCurrentPosition());
                telemetry.addData("left target", newLeftTarget);
                telemetry.addData("left current", FL.getCurrentPosition());
                telemetry.update();
            }

            StopDriving();

            // Turn off RUN_TO_POSITION
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


    public void StopDriving() {
        FL.setPower(0);
        FR.setPower(0);
    }

    public void TurnLeft() throws InterruptedException {
        telemetry.addData("LeftTurn", "Start");
        telemetry.update();
        FL.setPower(-1);
        FR.setPower(1);
        Thread.sleep(500);
        StopDriving();
        telemetry.addData("LeftTurn", "End");
        telemetry.update();
        sleep(1000);
    }

    public void TurnRight() throws InterruptedException {
//        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("RightTurn", "Start");
        telemetry.update();
        FL.setPower(1);
        FR.setPower(-1);
        Thread.sleep(500);
        StopDriving();
        telemetry.addData("RightTurn", "End");
        telemetry.update();
        sleep(1000);
    }
}
