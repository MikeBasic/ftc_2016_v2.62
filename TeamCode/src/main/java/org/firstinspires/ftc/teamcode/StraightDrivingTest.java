package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "Straight Driving Test", group = "Sigma6710")
@Disabled
public class StraightDrivingTest extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareSigma2016 robot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        double speed = 0.5;

        robot = new HardwareSigma2016();
        robot.init(hardwareMap);

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.update();
            idle();
        }

        // set mode
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        speed = 0.5;
        robot.frontLeftMotor.setPower(speed);
        robot.frontRightMotor.setPower(speed);
        robot.backRightMotor.setPower(speed);
        robot.backLeftMotor.setPower(speed);

        sleep(10*1000);

        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);

        speed = -0.5;
        robot.frontLeftMotor.setPower(speed);
        robot.frontRightMotor.setPower(speed);
        robot.backRightMotor.setPower(speed);
        robot.backLeftMotor.setPower(speed);

        sleep(10*1000);

        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }
}