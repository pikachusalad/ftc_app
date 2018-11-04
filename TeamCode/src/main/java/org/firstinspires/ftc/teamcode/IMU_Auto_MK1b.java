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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * {@link IMU_Auto_MK1b} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@Disabled
@Autonomous(name = "IMU_Auto_MK1b", group = "IMU1")
public class IMU_Auto_MK1b extends LinearOpMode
    {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object
    private BNO055IMU imu;
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor[] LeftMotors = new DcMotor[1];// = {FL};
    private DcMotor[] RightMotors = new DcMotor[1];// = {FR};
    private DcMotor[] AllMotors = new DcMotor[2];// = {FL, FR};
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // REV Hex HD 40:1
    static final double     WHEEL_DIAMETER_INCHES   = 3.54331;     // For figuring circumference
    static final double     WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV  / WHEEL_CIRCUMFERENCE_INCHES);
    static final double     MOTOR_POWER = 0.3;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override public void runOpMode() {

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");

        LeftMotors[0] = FL;
        RightMotors[0] = FR;
        AllMotors[0] = FL;
        AllMotors[1] = FR;


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        for (DcMotor m : LeftMotors)
            m.setDirection(DcMotor.Direction.REVERSE);
        for (DcMotor m : RightMotors)
            m.setDirection(DcMotor.Direction.FORWARD);
        for (DcMotor m : AllMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Wait until we're told to go
        waitForStart();
        runtime.reset();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        DriveEncoder(MOTOR_POWER, 24);
        sleep(1000);
        Turn(90, MOTOR_POWER);
        sleep(1000);
        DriveEncoder(MOTOR_POWER, 12);
        sleep(1000);
        Turn(90, MOTOR_POWER);
        sleep(1000);
        DriveEncoder(MOTOR_POWER, 24);
        Turn(-90, MOTOR_POWER);

        StopDriving();
    }

        public void DriveEncoder(double speed, double inches) {


            // Ensure that the opmode is still active
            if (opModeIsActive()) {
                int targetTicks = (int) (inches * COUNTS_PER_INCH);

                for (DcMotor m : AllMotors)
                    m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                for(DcMotor m : AllMotors) {
                    m.setTargetPosition(targetTicks);
                    m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                for (DcMotor m : AllMotors)
                    m.setPower(speed);

                // just keep looping while both motors are busy
                // stop if driverstation stop button pushed
                while (opModeIsActive() && (FL.isBusy() && FR.isBusy())) {
                    telemetry.addData("target ticks", targetTicks);
                    telemetry.addData("right current", FR.getCurrentPosition());
                    telemetry.addData("left current", FL.getCurrentPosition());
                    telemetry.update();
                }

                StopDriving();

                // Turn off RUN_TO_POSITION
                for (DcMotor m : AllMotors)
                    m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        public double GetCurrentZAngle() {
            Orientation currentAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            return currentAngles.thirdAngle;
        }

        public double GetAngleDifference(double startAngle) {
            double angleDifference = GetCurrentZAngle() - startAngle;

            if (angleDifference < -180)
                angleDifference += 360;
            else if (angleDifference > 180)
                angleDifference -=360;

            return angleDifference;
        }



        public void Turn(double targetAngleDifference, double power) {

            double startAngle = GetCurrentZAngle();
            boolean firstStep = false;
            boolean secondStep = false;

            if (targetAngleDifference < 0) {
                // turning right
                for (DcMotor m : RightMotors)
                    m.setPower(-power);
                for (DcMotor m : LeftMotors)
                    m.setPower(power);
                sleep(100);

                while (opModeIsActive() && GetAngleDifference(startAngle) > targetAngleDifference) {
                    telemetry.addData("target", targetAngleDifference);
                    telemetry.addData("current", GetAngleDifference(startAngle));
                    if (!secondStep && GetAngleDifference(startAngle) / targetAngleDifference > 0.75) {
                        for (DcMotor m : RightMotors)
                            m.setPower(-power/4);
                        for (DcMotor m : LeftMotors)
                            m.setPower(power/4);
                        secondStep = true;
                    } else if (!firstStep && GetAngleDifference(startAngle) / targetAngleDifference > 0.50) {
                        for (DcMotor m : RightMotors)
                            m.setPower(-power/2);
                        for (DcMotor m : LeftMotors)
                            m.setPower(power/2);
                        firstStep = true;
                    }
                    telemetry.update();
                }


            } else if (targetAngleDifference > 0) {
                // turning left
                for (DcMotor m : RightMotors)
                    m.setPower(power);
                for (DcMotor m : LeftMotors)
                    m.setPower(-power);
                sleep (100);
                while (opModeIsActive() && GetAngleDifference(startAngle) < targetAngleDifference) {
                    telemetry.addData("target", targetAngleDifference);
                    telemetry.addData("current", GetAngleDifference(startAngle));

                    if (!secondStep && GetAngleDifference(startAngle) / targetAngleDifference > 0.75) {
                        for (DcMotor m : RightMotors)
                            m.setPower(power/4);
                        for (DcMotor m : LeftMotors)
                            m.setPower(-power/4);
                        secondStep = true;
                    } else if (!firstStep && GetAngleDifference(startAngle) / targetAngleDifference > 0.50) {
                        for (DcMotor m : RightMotors)
                            m.setPower(power/2);
                        for (DcMotor m : LeftMotors)
                            m.setPower(-power/2);
                        firstStep = true;
                    }



                    telemetry.update();
                }
            } else {
                // is zero - not turning
                return;
            }

            StopDriving();

        }



        public void StopDriving() {
            for (DcMotor m : AllMotors)
                m.setPower(0);
        }
}
