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

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This is an OpMode that uses a hardware robot class
 */
@Autonomous(name = "IMU_Auto_MK1c", group = "IMU1")
public class IMU_Auto_MK1c extends LinearOpMode {

    // this is the motor power so when you make changes you can just make here
    // feel free to define multiple like FULL_POWER, HALF_POWER, etc.
    static final double DRIVE_SPEED = 0.3;

    @Override
    public void runOpMode() {

        // -------------------------------------------------------------------------------
        // create an instance of the hardware robot class, pass an instance of THIS OpMode
        SuspectTheRobot robot = new SuspectTheRobot(this);

        // call the initialization method
        robot.init();

        // -------------------------------------------------------------------------------
        // Wait until the start button is clicked!
        waitForStart();

        // -------------------------------------------------------------------------------
        // Start the logging of measured acceleration
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // -------------------------------------------------------------------------------
        // now do all of your driving and claiming depots and getting off landers or whatever
        // sleeps are not required
        // -------------------------------------------------------------------------------

        // drive forward about 24 inches
        robot.Drive(DRIVE_SPEED, 24);
        sleep(1000);

        // turn LEFT 90 degrees
        robot.Turn(90, DRIVE_SPEED);
        sleep(1000);

        // drive forward about 12 inches
        robot.Drive(DRIVE_SPEED, 12);
        sleep(1000);

        // turn LEFT 90 degrees
        robot.Turn(90, DRIVE_SPEED);
        sleep(1000);

        // Drive forward about 24 inches
        robot.Drive(DRIVE_SPEED, 24);
        sleep(1000);

        // Turn RIGHT 90 degrees
        robot.Turn(-90, DRIVE_SPEED);

        robot.StopDriving();
    }
}
