/*
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.text.DateFormat;
import java.util.Date;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.HardwareSigma2016.POWER_ADJ_STEP;
import static org.firstinspires.ftc.teamcode.HardwareSigma2016.PUSHER_L_IN;
import static org.firstinspires.ftc.teamcode.HardwareSigma2016.PUSHER_L_OUT;
import static org.firstinspires.ftc.teamcode.HardwareSigma2016.PUSHER_R_IN;
import static org.firstinspires.ftc.teamcode.HardwareSigma2016.PUSHER_STOP;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 * otherwise you would use: PushbotAutoDriveByEncoder;
 * <p>
 * This code requires that the drive Motors have been configured such that a positive
 * power command moves them forward, and causes the encoders to count UP.
 * <p>
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 * This is performed when the INIT button is pressed on the Driver Station.
 * This code assumes that the robot is stationary when the INIT button is pressed.
 * If this is not the case, then the INIT should be performed again.
 * <p>
 * Note: in this example, all angles are referenced to the initial coordinate frame set during the
 * the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 * <p>
 * The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 * which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 * This is consistent with the FTC field coordinate conventions set out in the document:
 * ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Red Corner", group = "Sigma6710")
//@Disabled
public class RedCorner extends LinearOpMode {

    static final double TARGET_WALL_DISTANCE_FORWARD = 10;  // ultrasound sensor reading for x inch away from wall
    static final double TARGET_WALL_DISTANCE_BACKWARD = 9;

    static final int RED_TRESHOLD = 5;
    static final int BLUE_TRESHOLD = 5;

    /* Declare OpMode members. */
    HardwareSigma2016 robot = null;

    // Sensor reading thread
    HighPriorityRunnerSigma2016 HighPriorityRunner;

    int ct0 = 0;
    int ct1 = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        String currentDateTimeString = DateFormat.getDateTimeInstance().format(new Date());
        System.out.println("--RedNear log@" + currentDateTimeString + "--");

        /* -------- initializations ---------- */
        /*
        * Initialize the standard drive system variables.
        * The init() method of the hardware class does most of the work here
        */
        robot = new HardwareSigma2016();
        robot.init(hardwareMap);

        // start the sensor reading thread
        HighPriorityRunner = new HighPriorityRunnerSigma2016(robot);
        HighPriorityRunner.start();

        // Ensure the robot is stationary, then reset the encoders and calibrate the gyro.
        robot.LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        robot.gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && robot.gyro.isCalibrating()) {
            robot.opModeLiveCheck = 1;

            sleep(50);
            idle();
        }

        robot.gyro.resetZAxisIntegrator();

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            robot.opModeLiveCheck = 1;

            telemetry.addData(">", "Robot Heading = %d", robot.gyro.getIntegratedZValue());
            telemetry.update();
            idle();
        }

        /* -------- driving to the predefined position ------- */
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // move forward
        gyroDrive(0.4 * robot.kMaxLinearSpeed,  // speed
                9.0,   // distance
                -5.0);// angle
        StopAllMotion();
        if (!opModeIsActive()) {
            HighPriorityRunner.go = false;
            return;
        }

        // wait the robot to be stable
        sleep(500);

        // shoot the 1st ball
        robot.flicker.setPower(1.0);
        sleep(800);
        robot.flicker.setPower(0);

        // release the 2nd ball
        robot.Storage.setPosition(robot.STORAGE_UP);
        sleep(600);
        robot.Storage.setPosition(robot.STORAGE_DOWN);

        // shoot the 2nd ball
        robot.flicker.setPower(1.0);
        sleep(800);
        robot.flicker.setPower(0);

        // turn toward beacons backward
        gyroTurn(0.6 * robot.kMaxLinearSpeed, // turn speed
                -130.0); // target heading
        StopAllMotion();
        if (!opModeIsActive()) {
            HighPriorityRunner.go = false;
            return;
        }

        // Drive toward beacons
        gyroDrive(1.0 * robot.kMaxLinearSpeed,  // speed
                -76,   // distance
                -133.0);// angle
        StopAllMotion();
        if (!opModeIsActive()) {
            HighPriorityRunner.go = false;
            return;
        }

        gyroTurn(0.5 * robot.kMaxLinearSpeed, // turn speed
                -165.0); // target heading
        StopAllMotion();
        if (!opModeIsActive()) {
            HighPriorityRunner.go = false;
            return;
        }

        UltraSonicReachTheWall(0.5 * robot.kMaxLinearSpeed, //speed
                -30, //distance
                -165.0); //angle (absolute)
        StopAllMotion();
        if (!opModeIsActive()) {
            HighPriorityRunner.go = false;
            return;
        }

        // 2nd line detection background light level sampling point
        robot.groundbrightness_test2 = robot.lineLightSensor.red() + robot.lineLightSensor.green() + robot.lineLightSensor.blue();

        gyroTurn(0.3 * robot.kMaxLinearSpeed, //speed
                -175.0); //angle
        StopAllMotion();
        if (!opModeIsActive()) {
            HighPriorityRunner.go = false;
            return;
        }

        // 3rd line detection background light level sampling point and calculate the average
        robot.groundbrightness_test3 = robot.lineLightSensor.red() + robot.lineLightSensor.green() + robot.lineLightSensor.blue();
        robot.groundbrightnessAVG = (robot.groundbrightness_test1 + robot.groundbrightness_test2 + robot.groundbrightness_test3) / 3;

        /* ------ ultrasonic wall tracker + white line detection ------- */
        // Drive forward to align with the wall and park at far line
        WallTrackingToWhiteLine(0.3 * robot.kMaxLinearSpeed, //speed
                -43, //distance
                true); //line tracking
        StopAllMotion();
        if (!opModeIsActive()) {
            HighPriorityRunner.go = false;
            return;
        }

        WallTrackingToWhiteLine(0.08 * robot.kMaxLinearSpeed, //speed
                18, //distance
                true); //line detection
        StopAllMotion();
        if (!opModeIsActive()) {
            HighPriorityRunner.go = false;
            return;
        }

        // run the beacon light color detection and button pushing procedure
        ColorDetectionAndButtonPushing();
        if (!opModeIsActive()) {
            StopAllMotion();
            HighPriorityRunner.go = false;
            return;
        }

        // Drive backward to detect the near line
        // pass the current white line without line detection
        WallTrackingToWhiteLine(0.5 * robot.kMaxLinearSpeed, //speed
                40.0, //distance
                false); //line detection
        StopAllMotion();
        if (!opModeIsActive()) {
            StopAllMotion();
            HighPriorityRunner.go = false;
            return;
        }

        // detect the near white line
        WallTrackingToWhiteLine(0.40 * robot.kMaxLinearSpeed, //speed
                13.0, //distance
                true); //color detection
        StopAllMotion();
        if (!opModeIsActive()) {
            HighPriorityRunner.go = false;
            return;
        }

        WallTrackingToWhiteLine(0.08 * robot.kMaxLinearSpeed, //speed
                -18.0, //distance
                true); //line detection
        StopAllMotion();
        if (!opModeIsActive()) {
            HighPriorityRunner.go = false;
            return;
        }

        // run the beacon light color detection and button pushing procedure
        ColorDetectionAndButtonPushing();
        if (!opModeIsActive()) {
            StopAllMotion();
            HighPriorityRunner.go = false;
            return;
        }

        /*------ drive to the center vortex ------*/
//        gyroDrive(1.0 * robot.kMaxLinearSpeed, //speed
//                -35.00, // distance
//                -300); // angle
//        StopAllMotion();
//        if (!opModeIsActive()) {
//            HighPriorityRunner.go = false;
//            return;
//        }
//
//        gyroTurn(0.6 * robot.kMaxLinearSpeed, //speed
//                -180);  //angle
//        StopAllMotion();
//        if (!opModeIsActive()) {
//            HighPriorityRunner.go = false;
//            return;
//        }
//
//        gyroDrive(1.0 * robot.kMaxLinearSpeed, //speed
//                62, //distance
//                -180);  //angle
//        StopAllMotion();
//        if (!opModeIsActive()) {
//            HighPriorityRunner.go = false;
//            return;
//        }

        // Finally, quit high priority thread
        HighPriorityRunner.go = false;
    }

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive(double speed,
                          double distance,
                          double angle) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftPower;
        double rightPower;
        double power = speed / robot.kMaxLinearSpeed;
        long curTime, timeInterval;
        long lastSpeedCheckTime = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.targetSpeed = speed;

            // reset encoder
            robot.LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // set mode
            robot.LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * robot.COUNTS_PER_INCH);
            newLeftTarget = robot.LeftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = robot.RightMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.LeftMotor.setTargetPosition(newLeftTarget);
            robot.RightMotor.setTargetPosition(newRightTarget);

            robot.LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion
            robot.LeftMotor.setPower(power);
            robot.RightMotor.setPower(power);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.LeftMotor.isBusy() && robot.RightMotor.isBusy())) {

                if (Math.abs(robot.LeftMotor.getCurrentPosition()
                        - robot.LeftMotor.getTargetPosition()) <= robot.ENCODER_TARGET_THRESHOLD) {
                    break;
                } else if (Math.abs(robot.RightMotor.getCurrentPosition()
                        - robot.RightMotor.getTargetPosition()) <= robot.ENCODER_TARGET_THRESHOLD) {
                    break;
                }

                // adjust relative power based on heading error.
                error = getError(angle);
                steer = getSteer(error, robot.P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftPower = power - steer;
                rightPower = power + steer;

                leftPower = clip(leftPower,
                        power - Math.abs(robot.maxLeftRightSpeedDifferentialAtDrive * power),
                        power + Math.abs(robot.maxLeftRightSpeedDifferentialAtDrive * power));
                rightPower = clip(rightPower,
                        power - Math.abs(robot.maxLeftRightSpeedDifferentialAtDrive * power),
                        power + Math.abs(robot.maxLeftRightSpeedDifferentialAtDrive * power));

                robot.LeftMotor.setPower(leftPower);
                robot.RightMotor.setPower(rightPower);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Targets L:R", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", robot.LeftMotor.getCurrentPosition(),
                        robot.RightMotor.getCurrentPosition());

                telemetry.addData("Power", "%5.2f:%5.2f", leftPower, rightPower);
                telemetry.update();

                // Monitor speed and adjust power if necessary
                curTime = System.currentTimeMillis();
                timeInterval = curTime - lastSpeedCheckTime;
                if (timeInterval > robot.SPEED_CHECK_INTERVAL) {
                    if (robot.currentSpeed < robot.targetSpeed * (1.0 - robot.SPEED_HYSTERESIS_RANGE)) {
                        power += POWER_ADJ_STEP;
                    } else if (robot.currentSpeed > robot.targetSpeed * (1.0 + robot.SPEED_HYSTERESIS_RANGE)) {
                        power -= POWER_ADJ_STEP;
                    }

                    power = Range.clip(power, 0.0, 1.0);

                    lastSpeedCheckTime = curTime;
                }

                robot.opModeLiveCheck = 1;
            }

            // Turn off RUN_TO_POSITION
            robot.LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.targetSpeed = 0;
        }
    }

    public void StopAllMotion() {
        // Stop all motion;
        robot.LeftMotor.setPower(0);
        robot.RightMotor.setPower(0);

//        robot.intake.setPower(0);
//        robot.flicker.setPower(0);
//        robot.CapLifter.setPower(0);
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        double power = speed / robot.kMaxLinearSpeed;
        double initialAngle = robot.gyro.getIntegratedZValue();
        double angleRange = angle - initialAngle;
        double powerAdjustingStepUp;
        double powerAdjustingStepDown;
        long curTime, timeInterval;
        long lastSpeedCheckTime = 0;

        // Notify high priority runner
        robot.targetSpeed = speed;

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(power, angle)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();

            robot.targetSpeed = speed * Math.abs((robot.gyro.getIntegratedZValue() - angle) / angleRange);
            if (robot.targetSpeed < robot.MIN_TURN_SPEED) {
                robot.targetSpeed = robot.MIN_TURN_SPEED;
            }
            powerAdjustingStepUp = robot.targetSpeed / robot.kMaxLinearSpeed / 10.0;
            powerAdjustingStepDown = robot.targetSpeed / robot.kMaxLinearSpeed / 3.0;

            curTime = System.currentTimeMillis();
            timeInterval = curTime - lastSpeedCheckTime;
            if (timeInterval > robot.SPEED_CHECK_INTERVAL) {
                if (robot.currentSpeed < robot.targetSpeed) {
                    power += powerAdjustingStepUp;
                } else if (robot.currentSpeed > robot.targetSpeed) {
                    power -= powerAdjustingStepDown;
                }

                power = Range.clip(power, 0.0, 1.0);
                lastSpeedCheckTime = curTime;

                System.out.printf("target angle=%.2f, curAngle=%d, power=%.2f, speed=%.2f",
                        angle,
                        robot.gyro.getIntegratedZValue(),
                        power,
                        robot.currentSpeed);
            }

            robot.opModeLiveCheck = 1;
        }

        robot.targetSpeed = 0;
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param power Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @return
     */
    boolean onHeading(double power, double angle) {
        double error;
        boolean onTarget = false;
        double leftPower;
        double rightPower;

        robot.LeftMotor.setMode(RUN_WITHOUT_ENCODER);
        robot.RightMotor.setMode(RUN_WITHOUT_ENCODER);

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= robot.TURN_THRESHOLD) {
            leftPower = 0.0;
            rightPower = 0.0;
            onTarget = true;
        } else {
            rightPower = power * Math.signum(error);
            leftPower = -rightPower;
        }

        // Send desired power to motors.
        robot.LeftMotor.setPower(leftPower);
        robot.RightMotor.setPower(rightPower);

        // Display it for the driver.
        telemetry.addData("Target:current", "%5.2f:5.2f", angle, robot.gyro.getIntegratedZValue());
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftPower, rightPower);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.gyro.getIntegratedZValue();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public boolean UltraSonicReachTheWall(double speed,
                                          double distance,
                                          double angle) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double error;
        double steer;
        double leftPower;
        double rightPower;
        double ultraSoundLevel, targetUS_Level;
        double power = speed / robot.kMaxLinearSpeed;
        long curTime, timeInterval;
        long lastSpeedCheckTime = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.targetSpeed = speed;

            // reset encoder
            robot.LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // set mode
            robot.LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * robot.COUNTS_PER_INCH);
            newLeftTarget = robot.LeftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = robot.RightMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.LeftMotor.setTargetPosition(newLeftTarget);
            robot.RightMotor.setTargetPosition(newRightTarget);

            robot.LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (distance < 0) {
                targetUS_Level = TARGET_WALL_DISTANCE_BACKWARD + 2;
            } else {
                targetUS_Level = TARGET_WALL_DISTANCE_FORWARD + 2;
            }

            // start motion.
            robot.LeftMotor.setPower(power);
            robot.RightMotor.setPower(power);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.LeftMotor.isBusy() && robot.RightMotor.isBusy())) {

                if (Math.abs(robot.LeftMotor.getCurrentPosition()
                        - robot.LeftMotor.getTargetPosition()) <= robot.ENCODER_TARGET_THRESHOLD) {
                    break;
                } else if (Math.abs(robot.RightMotor.getCurrentPosition()
                        - robot.RightMotor.getTargetPosition()) <= robot.ENCODER_TARGET_THRESHOLD) {
                    break;
                }

                // adjust relative power based on heading error.
                error = getError(angle);
                steer = getSteer(error, robot.P_WALL_APPROACHING_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftPower = power - steer;
                rightPower = power + steer;

                leftPower = clip(leftPower,
                        power - Math.abs(robot.maxLeftRightSpeedDifferentialAtDrive * power),
                        power + Math.abs(robot.maxLeftRightSpeedDifferentialAtDrive * power));
                rightPower = clip(rightPower,
                        power - Math.abs(robot.maxLeftRightSpeedDifferentialAtDrive * power),
                        power + Math.abs(robot.maxLeftRightSpeedDifferentialAtDrive * power));

                robot.LeftMotor.setPower(leftPower);
                robot.RightMotor.setPower(rightPower);

                if (distance < 0) {
                    ultraSoundLevel = robot.ultra_back.getUltrasonicLevel();
                } else {
                    ultraSoundLevel = robot.ultra_front.getUltrasonicLevel();
                }

                ct1++;
                if (ct1 > 100) {
                    ct1 = 0;
                    System.out.println("-- wall approaching -- ultrasound level = " + ultraSoundLevel);
                }

                // handles abnormal ultrasonic reading
                if (ultraSoundLevel == 0) {
                    // stop the robot
                    robot.LeftMotor.setPower(0);
                    robot.RightMotor.setPower(0);

                    System.out.println("--Sigma2016-- abnormal -- ultrasound level=" + ultraSoundLevel);

                    sleep(100);
                    idle();
                } else if (ultraSoundLevel == 255) {
                    // error reading. Ignore.
                    continue;
                } else if (ultraSoundLevel <= targetUS_Level) {

                    System.out.println("Sigma2016 -- wall reached, ultrasound level=" + ultraSoundLevel);

                    // reached the wall. stop.
                    break;
                }

                // Monitor speed and adjust power if necessary
                curTime = System.currentTimeMillis();
                timeInterval = curTime - lastSpeedCheckTime;
                if (timeInterval > robot.SPEED_CHECK_INTERVAL) {
                    if (robot.currentSpeed < robot.targetSpeed) {
                        power += POWER_ADJ_STEP;
                    } else if (robot.currentSpeed > robot.targetSpeed) {
                        power -= POWER_ADJ_STEP;
                    }

                    power = Range.clip(power, 0.0, 1.0);

                    lastSpeedCheckTime = curTime;
                }

                robot.opModeLiveCheck = 1;
            }

            // Turn off RUN_TO_POSITION
            robot.LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.targetSpeed = 0;
        }

        return (true);
    }

    /**
     * Method to track along a wall using an ultrasonic sensor
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired distance (timeout if no white line found)
     * 2) Driver stops the opmode running.
     * 3) White line on the ground is detected and aligned by the light sensors
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     */
    public void WallTrackingToWhiteLine(double speed,
                                        double distance,
                                        boolean bLineDetection) {
        long curTime, timeInterval;
        long previousRunTime = 0, lastSpeedCheckTime = 0;
        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double targetWallDistance;
        double power = speed / robot.kMaxLinearSpeed;
        int expectedRunInterval = 10; // ms
        double error, lastErr = 0;
        double steer = 0.0;
        double leftPower;
        double rightPower;
        double ultraSoundLevel, angleOffset;
        double targetAngleOffset, angleSteer;
        double minPower, maxPower;
        double lastLeftPower = 0.0, lastRightPower = 0.0;
        double powerAdjustingStepUp = 0.01;
        double powerAdjustingStepDown = 0.03;
//        double powerAdjustingStep = speed * robot.kMaxLinearSpeed / 10.0;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.targetSpeed = speed;
            HighPriorityRunner.bLineDetection = bLineDetection;
            robot.maxLineBrightnessReading = 0;

            // reset encoder
            robot.LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // set mode
            robot.LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * robot.COUNTS_PER_INCH);
            newLeftTarget = robot.LeftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = robot.RightMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.LeftMotor.setTargetPosition(newLeftTarget);
            robot.RightMotor.setTargetPosition(newRightTarget);

            robot.LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (distance < 0) {
                targetWallDistance = TARGET_WALL_DISTANCE_BACKWARD;
            } else {
                targetWallDistance = TARGET_WALL_DISTANCE_FORWARD;
            }

            // start motion.
            robot.LeftMotor.setPower(power);
            robot.RightMotor.setPower(power);

            lastLeftPower = power;
            lastRightPower = power;

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.LeftMotor.isBusy() && robot.RightMotor.isBusy())) {

                curTime = System.currentTimeMillis();
                if (previousRunTime != 0) {
                    timeInterval = curTime - previousRunTime;

                    if (timeInterval > expectedRunInterval * 2) {
                        System.out.println("Sigma2016 -- Blue While did not run for " + timeInterval + "ms");
                    }
                }
                previousRunTime = curTime;

                // if motor reaches its encoder destination, stop.
                if (Math.abs(robot.LeftMotor.getCurrentPosition()
                        - robot.LeftMotor.getTargetPosition()) <= robot.ENCODER_TARGET_THRESHOLD) {
                    break;
                } else if (Math.abs(robot.RightMotor.getCurrentPosition()
                        - robot.RightMotor.getTargetPosition()) <= robot.ENCODER_TARGET_THRESHOLD) {
                    break;
                }

                minPower = -Math.abs(power);
                maxPower = Math.abs(power);

                if (distance < 0) {
                    ultraSoundLevel = robot.ultra_back.getUltrasonicLevel();
                } else {
                    ultraSoundLevel = robot.ultra_front.getUltrasonicLevel();
                }

                error = ultraSoundLevel - targetWallDistance;

                // get angle offset of the wall
                angleOffset = robot.gyro.getIntegratedZValue();

                ct0++;
                if (error != lastErr) {
                    ct0 = 51;
                }

                lastErr = error;

                if (ct0 > 50) {
                    ct0 = 0;

                    System.out.println("--Sigma2016-- ultrasoniclevel=" + ultraSoundLevel
                            + " error=" + error
                            + " angleOffset=" + angleOffset
                            + " distance=" + distance);
                }

                if (ultraSoundLevel == 255) {
                    // error reading. Ignore.
                    continue;
                }

                // adjust angle pointing based on ultrasound reading.
                if (Math.abs(error) >= 2) {
                    // distance off by more than 2. Need steep 10 degree angle driving back to expected distance
                    targetAngleOffset = -180 - Math.signum(distance) * Math.signum(error) * 4.0;

                    angleSteer = targetAngleOffset - angleOffset;

                    steer = Math.signum(distance) * angleSteer * 0.04;

                    leftPower = power - steer;
                    rightPower = power + steer;

                    leftPower = Range.clip(leftPower, minPower, maxPower);
                    rightPower = Range.clip(rightPower, minPower, maxPower);

                    if (Math.abs(lastLeftPower - leftPower) > 0.01) {
                        robot.LeftMotor.setPower(leftPower);
                        lastLeftPower = leftPower;
                    }

                    if (Math.abs(lastRightPower - rightPower) > 0.01) {
                        robot.RightMotor.setPower(rightPower);
                        lastRightPower = rightPower;
                    }

                    if (ct0 == 0) {
                        System.out.println("--Sigma2016-- error=" + String.format(Double.toString(error), "%5.2f")
                                + " leftPower=" + String.format(Double.toString(leftPower), "%5.2f")
                                + " rightPower=" + String.format(Double.toString(rightPower), "%5.2f")
                                + " curAngle=" + angleOffset
                                + " targetAngle=" + targetAngleOffset);
                    }
                } else if (Math.abs(error) >= 1) {

                    // distance off by 1. Need mild angle driving back to expected distance
                    // 3.0 is the expected front/back ultrasound sensor difference
                    targetAngleOffset = -180 - Math.signum(distance) * Math.signum(error) * 2.0;

                    angleSteer = targetAngleOffset - angleOffset;

                    steer = Math.signum(distance) * angleSteer * 0.02;

                    leftPower = power - steer;
                    rightPower = power + steer;

                    leftPower = Range.clip(leftPower, minPower, maxPower);
                    rightPower = Range.clip(rightPower, minPower, maxPower);

                    if (Math.abs(lastLeftPower - leftPower) > 0.01) {
                        robot.LeftMotor.setPower(leftPower);
                        lastLeftPower = leftPower;
                    }

                    if (Math.abs(lastRightPower - rightPower) > 0.01) {
                        robot.RightMotor.setPower(rightPower);
                        lastRightPower = rightPower;
                    }

                    if (ct0 == 0) {
                        System.out.println("--Sigma2016-- error=" + String.format(Double.toString(error), "%5.2f")
                                + " leftPower=" + String.format(Double.toString(leftPower), "%5.2f")
                                + " rightPower=" + String.format(Double.toString(rightPower), "%5.2f")
                                + " curAngle=" + angleOffset
                                + " targetAngle=" + targetAngleOffset);
                    }
                } else {

                    // distance on target. Need to keep 0 angle offset
                    // 3.0 is the expected front/back ultrasound sensor difference
                    targetAngleOffset = -180.0;

                    angleSteer = targetAngleOffset - angleOffset;

                    steer = Math.signum(distance) * angleSteer * 0.04;

                    leftPower = power - steer;
                    rightPower = power + steer;

                    leftPower = Range.clip(leftPower, minPower, maxPower);
                    rightPower = Range.clip(rightPower, minPower, maxPower);

                    if (Math.abs(lastLeftPower - leftPower) > 0.01) {
                        robot.LeftMotor.setPower(leftPower);
                        lastLeftPower = leftPower;
                    }

                    if (Math.abs(lastRightPower - rightPower) > 0.01) {
                        robot.RightMotor.setPower(rightPower);
                        lastRightPower = rightPower;
                    }

                    if (ct0 == 0) {
                        System.out.println("--Sigma2016-- error=" + String.format(Double.toString(error), "%5.2f")
                                + " leftPower=" + String.format(Double.toString(leftPower), "%5.2f")
                                + " rightPower=" + String.format(Double.toString(rightPower), "%5.2f")
                                + " curAngle=" + angleOffset
                                + " targetAngle=" + targetAngleOffset);
                    }
                }

                // Monitor speed and adjust power if necessary
                if (curTime - lastSpeedCheckTime > robot.SPEED_CHECK_INTERVAL) {

                    if (Math.abs(robot.targetSpeed) >= 0.1 * robot.kMaxLinearSpeed) {
                        if (robot.currentSpeed < robot.targetSpeed) {
                            power += powerAdjustingStepUp;
                        } else if (robot.currentSpeed > robot.targetSpeed) {
                            power -= powerAdjustingStepDown;
                        }
                    }
                    else
                    {
                        // Only increase power when robot does not move
                        if (robot.currentSpeed < 0.8) {
                            power += powerAdjustingStepUp;
                        } else if (robot.currentSpeed > robot.targetSpeed) {
                            power -= powerAdjustingStepDown;
                        }
                    }

                    power = Range.clip(power, 0.0, 1.0);
                }

                if (robot.maxLineBrightnessReading > robot.groundbrightnessAVG * robot.CENTER_LIGHT_THRESH) {
                    System.out.println("Sigma2016 ---> Ground Brightness:: " + robot.groundbrightnessAVG
                            + " DETECTED Light Level:: " + robot.maxLineBrightnessReading);

                    break;
                }

                robot.opModeLiveCheck = 1;
            }
        }

        // Turn off RUN_TO_POSITION
        robot.LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.targetSpeed = 0;
        HighPriorityRunner.bLineDetection = false;
        robot.maxLineBrightnessReading = 0;
    }

    // detect the color and push the red button.
    public void ColorDetectionAndButtonPushing() {

        ElapsedTime holdTimer = new ElapsedTime();
        double holdTime = 2;  // 2 second timeout
        int red, green, blue;
        int redCheck = 0, blueCheck = 0;

        robot.beaconColorSensor.enableLed(false); //led OFF

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {

            red = robot.beaconColorSensor.red();
            green = robot.beaconColorSensor.green();
            blue = robot.beaconColorSensor.blue();

            System.out.println("--BlueNear log-- R:G:B = " + red + ":" + green + ":" + blue);

            if ((red > blue + 10) && (red > green + 10)) {
                redCheck++;
            } else {
                redCheck = 0;
            }

            if ((blue > red) && (blue > green)) {
                blueCheck++;
            } else {
                blueCheck = 0;
            }

            telemetry.addData("ColorRGB:: ", "%d %d %d", red, green, blue);
            telemetry.addData("ColorRC&BC :: ", "%d %d", redCheck, blueCheck);
            telemetry.update();

            // red color detected
            if (redCheck > RED_TRESHOLD) {

                // We are red team
                robot.pusherL.setPosition(robot.PUSHER_L_OUT);
                //wait servo to finish
                sleep(1600);

                // Retrieve the pusher
                robot.pusherL.setPosition(robot.PUSHER_L_IN);
                robot.pusherRetractionStartTime = System.currentTimeMillis();
                sleep(300);

                System.out.println("--Sigma2016-- red light detected and red button pushed. redCheck=" + redCheck + " blueCheck=" + blueCheck);
                break;
            }

            // blue color detected
            if (blueCheck > BLUE_TRESHOLD) {

                // We are the blue team
                robot.pusherR.setPosition(robot.PUSHER_R_OUT);
                //wait servo to finish
                sleep(1600);

                // Retrieve the pusher
                robot.pusherR.setPosition(robot.PUSHER_R_IN);
                robot.pusherRetractionStartTime = System.currentTimeMillis();
                sleep(300);

                System.out.println("--Sigma2016-- blue light detected and red button pushed. blueCheck=" + blueCheck + " redCheck=" + redCheck);
                break;
            }

            robot.opModeLiveCheck = 1;

            sleep(10);
            idle();
        }
    }
}