/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * a legacy (NXT-compatible) Hitechnic Color Sensor v2.
 * It assumes that the color sensor is configured with a name of "sensor_color".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name = "MotorTest", group = "Sigma6710")
@Disabled
public class MotorTestSigma2016 extends LinearOpMode {

    HardwareSigma2016 robot = null;
    HighPriorityRunnerSigma2016 HighPriorityRunner;

    @Override
    public void runOpMode() throws InterruptedException {

        long timeStart;

        robot = new HardwareSigma2016();
        robot.init(hardwareMap);

        // wait for the start button to be pressed.
        waitForStart();

//        robot.LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.LeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.LeftMotor.setPower(0.5);
//        sleep(500);
//        robot.LeftMotor.setPower(0.0);
//        sleep(3000);
//
//        robot.RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.RightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.RightMotor.setPower(0.5);
//        sleep(500);
//        robot.RightMotor.setPower(0.0);
//        sleep(3000);
//
//        robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.intake.setPower(0.5);
//        sleep(500);
//        robot.intake.setPower(0.0);
//        sleep(3000);
//
//        robot.flicker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.flicker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.flicker.setPower(0.5);
//        sleep(500);
//        robot.flicker.setPower(0.0);
//        sleep(3000);
//
//        robot.CapLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.CapLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.CapLifter.setPower(0.5);
//        sleep(500);
//        robot.CapLifter.setPower(0.0);
//        sleep(3000);

        /* -------- speed test -------- */
        // start the high priority runner
        HighPriorityRunner = new HighPriorityRunnerSigma2016(robot);
        HighPriorityRunner.start();

        robot.LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.LeftMotor.setPower(1.0);

        robot.RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.RightMotor.setPower(1.0);

        timeStart = System.currentTimeMillis();
        while(System.currentTimeMillis() - timeStart < 80000)
        {
            telemetry.addData("velocity speed = ", "%.2f in/s", robot.currentSpeed);
            telemetry.addData("enc = ", "%d %d", HighPriorityRunner.lastLeftEncoderPosition, HighPriorityRunner.lastRightEncoderPosition);
            telemetry.addData("angle speed = ", "%.2f d/s", robot.currentAngleSpeed);
            telemetry.update();

            sleep(10);
        }

        robot.LeftMotor.setPower(0.0);
        robot.RightMotor.setPower(0.0);

        HighPriorityRunner.go = false;

        /* -------- encoder reading per round test -------- */
//        robot.LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.LeftMotor.setTargetPosition(1310);  // one whole turn
//        robot.LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.LeftMotor.setPower(0.05);
//
//        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
//        while (opModeIsActive()) {
//
//            telemetry.addData("Motor Pos: ", "%d", robot.LeftMotor.getCurrentPosition());
//
//            telemetry.update();
//
//            if(Math.abs(robot.LeftMotor.getCurrentPosition() - robot.LeftMotor.getTargetPosition()) <= 10)
//            {
//                robot.LeftMotor.setPower(0.0);
//            }
//        }
    }
}
