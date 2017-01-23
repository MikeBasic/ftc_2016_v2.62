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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.text.DateFormat;
import java.util.Date;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static org.firstinspires.ftc.teamcode.HardwareSigma2016.PUSHER_L_IN;
import static org.firstinspires.ftc.teamcode.HardwareSigma2016.PUSHER_L_OUT;
import static org.firstinspires.ftc.teamcode.HardwareSigma2016.PUSHER_R_IN;
import static org.firstinspires.ftc.teamcode.HardwareSigma2016.PUSHER_R_OUT;
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

@Autonomous(name = "Blue Near Auto Op Sigma 2016", group = "Sigma6710")
//@Disabled
public class BlueNearAutoOpSigma2016 extends LinearOpMode {

    static final double COUNTS_PER_MOTOR_REV = 1310 * 1.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1 / 1.6;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.9;     // Nominal speed for better accuracy.
    static final double P_DRIVE_COEFF = 0.05;     // Larger is more responsive, but also less stable

    static final double TURN_SPEED = 0.25;     // Nominal half speed for better accuracy.
    static final double TURN_THRESHOLD = 2;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.05;     // Larger is more responsive, but also less stable
    static final double MIN_TURN_SPEED = 0.2;     // Larger is more responsive, but also less stable with over turn

    static final double maxLeftRightSpeedDifferentialAtDrive = 0.5;

    static final double WALL_APPROACHING_SPEED = 0.6;
    static final double P_WALL_APPROACHING_COEFF = 0.05;

    static final double LINE_DETECTION_SPEED = 0.13;
    static final double WALL_TRAVELING_SPEED = 0.5;
    static final double P_WALL_TRACKING_COEFF_FINE = 0.025;// Larger is more responsive, but also less stable
    static final double P_WALL_TRACKING_COEFF_COARSE = 0.05;// Larger is more responsive, but also less stable

    static final double TARGET_WALL_DISTANCE_FORWARD = 7;  // ultrasound sensor reading for x inch away from wall
    static final double TARGET_WALL_DISTANCE_BACKWARD = 7;

    static final int ENCODER_TARGET_THRESHOLD = 10;

    static final int RED_TRESHOLD = 5;
    static final int BLUE_TRESHOLD = 5;

    /* Declare OpMode members. */
    HardwareSigma2016 robot = null;
    ModernRoboticsI2cGyro gyro = null;                    // Additional Gyro device

    // Sensor reading thread
    ReadSensorsSigma2016 sensorReadingThread;

    int ct0 = 0;
    int ct1 = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        String currentDateTimeString = DateFormat.getDateTimeInstance().format(new Date());
        System.out.println("--BlueNear log@" + currentDateTimeString + "--");

        /* -------- initializations ---------- */
        /*
        * Initialize the standard drive system variables.
        * The init() method of the hardware class does most of the work here
        */
        robot = new HardwareSigma2016();
        robot.init(hardwareMap);

        // start the sensor reading thread
        sensorReadingThread = new ReadSensorsSigma2016(robot);
        sensorReadingThread.start();

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro.isCalibrating()) {
            sleep(50);
            idle();
        }

        gyro.resetZAxisIntegrator();

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
            idle();
        }

        /* -------- driving to the predefined position ------- */
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        gyroDrive(DRIVE_SPEED, 79, -45.0);// Drive FWD 81 inches along -50 degree
        StopAllMotion();
        if (!opModeIsActive()) {
            return;
        }

        // Turn to -20 degree. Make the turn coeff huge so full TURN_SPEED power will be used.
        gyroTurn(TURN_SPEED, -20.0, P_TURN_COEFF);
        StopAllMotion();
        if (!opModeIsActive()) {
            return;
        }

        UltraSonicReachTheWall(WALL_APPROACHING_SPEED, 60, -15.0);
        StopAllMotion();
        if (!opModeIsActive()) {
            return;
        }

        // 2nd line detection background light level sampling point
        robot.groundbrightness_test2 = robot.lineLightSensor.red() + robot.lineLightSensor.green() + robot.lineLightSensor.blue();

        // Turn to 0.0 degree with speed TURN_SPEED. Make the turn coeff huge so full TURN_SPEED power will be used.
        // Otherwise the robot could stuck because only a portion of TURN_SPEED power is applied.
        gyroTurn(TURN_SPEED, -5.0, P_TURN_COEFF);
        StopAllMotion();
        if (!opModeIsActive()) {
            return;
        }

        // 3rd line detection background light level sampling point and calculate the average
        robot.groundbrightness_test3 = robot.lineLightSensor.red() + robot.lineLightSensor.green() + robot.lineLightSensor.blue();
        robot.groundbrightnessAVG = (robot.groundbrightness_test1 + robot.groundbrightness_test2 + robot.groundbrightness_test3) / 3;

        /* ------ ultrasonic wall tracker + white line detection ------- */
        // Drive forward to align with the wall and park at far line
//        WallTrackingToWhiteLine(0.4, 14, false);
//        StopAllMotion();
//        if (!opModeIsActive()) {
//            return;
//        }

        WallTrackingToWhiteLine(0.25, 54, true);
        StopAllMotion();
        if (!opModeIsActive()) {
            return;
        }

        WallTrackingToWhiteLine(LINE_DETECTION_SPEED, -18, true);
        StopAllMotion();
        if (!opModeIsActive()) {
            return;
        }

        // run the beacon light color detection and button pushing procedure
        ColorDetectionAndButtonPushing();
        if (!opModeIsActive()) {
            StopAllMotion();
            return;
        }

        // Drive backward to detect the near line
        // pass the current white line without line detection
        WallTrackingToWhiteLine(0.5, -35.0, false);
        StopAllMotion();
        if (!opModeIsActive()) {
            StopAllMotion();
            return;
        }

        // detect the near white line
        WallTrackingToWhiteLine(0.2, -23.0, true);
        StopAllMotion();
        if (!opModeIsActive()) {
            return;
        }

        WallTrackingToWhiteLine(0.13 , 18.0, true);
        StopAllMotion();
        if (!opModeIsActive()) {
            return;
        }

        // run the beacon light color detection and button pushing procedure
        ColorDetectionAndButtonPushing();
        if (!opModeIsActive()) {
            StopAllMotion();
            return;
        }

        /*------ drive to the center vortex ------*/
        gyroDrive(DRIVE_SPEED, 35.00, 50.0); // 30 degree
        if (!opModeIsActive()) {
            StopAllMotion();
            return;
        }

        gyroTurn(TURN_SPEED, -120, P_TURN_COEFF); // turn to 90 degree
        StopAllMotion();
        if (!opModeIsActive()) {
            return;
        }
        robot.flicker.setPower(1.0);
        sleep(800);
        StopAllMotion();

        robot.intake.setPower(-0.5);
        sleep(1500);
        if (!opModeIsActive()) {
            return;
        }
        StopAllMotion();

        robot.flicker.setPower(1.0);
        sleep(800);
        StopAllMotion();

//        gyroTurn(TURN_SPEED, -40, P_TURN_COEFF);  // -40 degree
//        gyroDrive(DRIVE_SPEED, -41, -40);         // -40 degree, 41 inch
//        StopAllMotion();
//        if (!opModeIsActive()) {
//            return;
//        }

        // Finally, stop
//        StopAllMotion();
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
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // reset encoder
            robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // set mode
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeftMotor.setMode(RUN_WITHOUT_ENCODER);
            robot.backRightMotor.setMode(RUN_WITHOUT_ENCODER);

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = robot.frontLeftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = robot.frontRightMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.frontLeftMotor.setTargetPosition(newLeftTarget);
            robot.frontRightMotor.setTargetPosition(newRightTarget);

            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // determine back motor's direction
            if (distance < 0) {
                if (robot.backLeftMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
                    robot.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                } else {
                    robot.backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                }

                if (robot.backRightMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
                    robot.backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                } else {
                    robot.backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                }
            }

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.frontLeftMotor.setPower(speed);
            robot.frontRightMotor.setPower(speed);
            robot.backRightMotor.setPower(speed);
            robot.backLeftMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy())) {

                if (Math.abs(robot.frontLeftMotor.getCurrentPosition()
                        - robot.frontLeftMotor.getTargetPosition()) <= ENCODER_TARGET_THRESHOLD) {
                    break;
                } else if (Math.abs(robot.frontRightMotor.getCurrentPosition()
                        - robot.frontRightMotor.getTargetPosition()) <= ENCODER_TARGET_THRESHOLD) {
                    break;
                }

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                leftSpeed = Range.clip(leftSpeed,
                        speed - Math.abs(maxLeftRightSpeedDifferentialAtDrive * speed),
                        speed + Math.abs(maxLeftRightSpeedDifferentialAtDrive * speed));
                rightSpeed = Range.clip(rightSpeed,
                        speed - Math.abs(maxLeftRightSpeedDifferentialAtDrive * speed),
                        speed + Math.abs(maxLeftRightSpeedDifferentialAtDrive * speed));

                robot.frontLeftMotor.setPower(leftSpeed);
                robot.frontRightMotor.setPower(rightSpeed);
                robot.backLeftMotor.setPower(leftSpeed);
                robot.backRightMotor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Targets L:R", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", robot.frontLeftMotor.getCurrentPosition(),
                        robot.frontRightMotor.getCurrentPosition());

                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Turn off RUN_TO_POSITION
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (distance < 0) {
                if (robot.backLeftMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
                    robot.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                } else {
                    robot.backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                }

                if (robot.backRightMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
                    robot.backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                } else {
                    robot.backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                }
            }
        }
    }

    public void StopAllMotion() {
        // Stop all motion;
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.intake.setPower(0);
        robot.flicker.setPower(0);
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
    public void gyroTurn(double speed, double angle, double turnCoeff) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, turnCoeff)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        robot.frontLeftMotor.setMode(RUN_WITHOUT_ENCODER);
        robot.frontRightMotor.setMode(RUN_WITHOUT_ENCODER);
        robot.backLeftMotor.setMode(RUN_WITHOUT_ENCODER);
        robot.backRightMotor.setMode((RUN_WITHOUT_ENCODER));

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= TURN_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            if (Math.abs(rightSpeed) < MIN_TURN_SPEED) {
                rightSpeed = Math.abs(rightSpeed) * MIN_TURN_SPEED / rightSpeed;
            }

            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.frontLeftMotor.setPower(leftSpeed);
        robot.frontRightMotor.setPower(rightSpeed);
        robot.backLeftMotor.setPower(leftSpeed);
        robot.backRightMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target:current", "%5.2f:5.2f", angle, gyro.getIntegratedZValue());
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

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
        robotError = targetAngle - gyro.getIntegratedZValue();
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
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;
        double ultraSoundLevel, targetUS_Level;
        double lightLevel;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // reset encoder
            robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // set mode
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeftMotor.setMode(RUN_WITHOUT_ENCODER);
            robot.backRightMotor.setMode(RUN_WITHOUT_ENCODER);

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = robot.frontLeftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = robot.frontRightMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.frontLeftMotor.setTargetPosition(newLeftTarget);
            robot.frontRightMotor.setTargetPosition(newRightTarget);

            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (distance < 0) {
                if (robot.backLeftMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
                    robot.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                } else {
                    robot.backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                }

                if (robot.backRightMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
                    robot.backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                } else {
                    robot.backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                }

                targetUS_Level = TARGET_WALL_DISTANCE_BACKWARD + 2;
            } else {
                targetUS_Level = TARGET_WALL_DISTANCE_FORWARD + 2;
            }

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.frontLeftMotor.setPower(speed);
            robot.frontRightMotor.setPower(speed);
            robot.backRightMotor.setPower(speed);
            robot.backLeftMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy())) {

                if (Math.abs(robot.frontLeftMotor.getCurrentPosition() - robot.frontLeftMotor.getTargetPosition()) <= ENCODER_TARGET_THRESHOLD) {
                    break;
                } else if (Math.abs(robot.frontRightMotor.getCurrentPosition() - robot.frontRightMotor.getTargetPosition()) <= ENCODER_TARGET_THRESHOLD) {
                    break;
                }

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_WALL_APPROACHING_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                leftSpeed = Range.clip(leftSpeed,
                        speed - Math.abs(maxLeftRightSpeedDifferentialAtDrive * speed),
                        speed + Math.abs(maxLeftRightSpeedDifferentialAtDrive * speed));
                rightSpeed = Range.clip(rightSpeed,
                        speed - Math.abs(maxLeftRightSpeedDifferentialAtDrive * speed),
                        speed + Math.abs(maxLeftRightSpeedDifferentialAtDrive * speed));

                robot.frontLeftMotor.setPower(leftSpeed);
                robot.frontRightMotor.setPower(rightSpeed);
                robot.backLeftMotor.setPower(leftSpeed);
                robot.backRightMotor.setPower(rightSpeed);

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
                    robot.frontLeftMotor.setPower(0);
                    robot.frontRightMotor.setPower(0);
                    robot.backLeftMotor.setPower(0);
                    robot.backRightMotor.setPower(0);

                    System.out.println("--BlueNear log-- abnormal -- ultrasound level=" + ultraSoundLevel);

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
            }

            // Turn off RUN_TO_POSITION
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (distance < 0) {
                if (robot.backLeftMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
                    robot.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                } else {
                    robot.backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                }

                if (robot.backRightMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
                    robot.backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                } else {
                    robot.backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                }
            }
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
        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double error, lastErr=0;
        double steer = 0;
        double leftSpeed;
        double rightSpeed;
        double ultraSoundLevel, angleOffset;
        int lightlevel = 0;
        double targetWallDistance, targetAngleOffset, angleSteer;
        long curTime, timeInterval;
        long previousRunTime = 0;
        int expectedRunInterval = 10; // millisecond
        double minSpeed = -Math.abs(speed);
        double maxSpeed = Math.abs(speed);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // reset encoder
            robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // set mode
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeftMotor.setMode(RUN_WITHOUT_ENCODER);
            robot.backRightMotor.setMode(RUN_WITHOUT_ENCODER);

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = robot.frontLeftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = robot.frontRightMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.frontLeftMotor.setTargetPosition(newLeftTarget);
            robot.frontRightMotor.setTargetPosition(newRightTarget);

            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (distance < 0) {
                if (robot.backLeftMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
                    robot.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                } else {
                    robot.backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                }

                if (robot.backRightMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
                    robot.backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                } else {
                    robot.backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                }

                targetWallDistance = TARGET_WALL_DISTANCE_BACKWARD;
            } else {
                targetWallDistance = TARGET_WALL_DISTANCE_FORWARD;
            }

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.frontLeftMotor.setPower(speed);
            robot.frontRightMotor.setPower(speed);
            robot.backRightMotor.setPower(speed);
            robot.backLeftMotor.setPower(speed);

            if (bLineDetection) {
                // turn on line light sensor reading
                sensorReadingThread.SetCenterLightSensorRead(true);
            }

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.frontLeftMotor.isBusy() && robot.frontRightMotor.isBusy())) {

                if (Math.abs(robot.frontLeftMotor.getCurrentPosition() - robot.frontLeftMotor.getTargetPosition()) <= ENCODER_TARGET_THRESHOLD) {
                    break;
                } else if (Math.abs(robot.frontRightMotor.getCurrentPosition() - robot.frontRightMotor.getTargetPosition()) <= ENCODER_TARGET_THRESHOLD) {
                    break;
                }

                curTime = System.currentTimeMillis();
                if (previousRunTime != 0) {
                    timeInterval = curTime - previousRunTime;

                    if (timeInterval > expectedRunInterval * 2) {
                        System.out.println("Sigma2016 -- Blue While did not run for " + timeInterval + "ms");
                    }
                }
                previousRunTime = curTime;

                if (distance < 0) {
                    ultraSoundLevel = robot.ultra_back.getUltrasonicLevel();
                } else {
                    ultraSoundLevel = robot.ultra_front.getUltrasonicLevel();
                }

                error = ultraSoundLevel - targetWallDistance;

                // get angle offset of the wall
                angleOffset = gyro.getIntegratedZValue();

                ct0++;
                if (error != lastErr)
                {
                    ct0 = 51;
                }

                lastErr = error;

                if (ct0 > 50) {
                    ct0 = 0;

                    System.out.println("--BlueNear log-- ultrasoniclevel=" + ultraSoundLevel
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
                    targetAngleOffset = 0.0 - Math.signum(distance) * Math.signum(error) * 4.0;

                    angleSteer = targetAngleOffset - angleOffset;

                    steer = Math.signum(distance) * angleSteer * 0.04;

                    leftSpeed = speed - steer;
                    rightSpeed = speed + steer;

                    leftSpeed = Range.clip(leftSpeed, minSpeed, maxSpeed);
                    rightSpeed = Range.clip(rightSpeed, minSpeed, maxSpeed);

                    robot.frontLeftMotor.setPower(leftSpeed);
                    robot.frontRightMotor.setPower(rightSpeed);
                    robot.backLeftMotor.setPower(leftSpeed);
                    robot.backRightMotor.setPower(rightSpeed);

                    if (ct0 == 0) {
                        System.out.println("--BlueNear log-- error=" + String.format(Double.toString(error), "%5.2f")
                                + " leftspeed=" + String.format(Double.toString(leftSpeed), "%5.2f")
                                + " rightSpeed=" + String.format(Double.toString(rightSpeed), "%5.2f")
                                + " curAngle=" + angleOffset
                                + " targetAngle=" + targetAngleOffset);
                    }
                } else if (Math.abs(error) >= 1) {

                    // distance off by 1. Need mild angle driving back to expected distance
                    // 3.0 is the expected front/back ultrasound sensor difference
                    targetAngleOffset = 0.0 - Math.signum(distance) * Math.signum(error) * 2.0;

                    angleSteer = targetAngleOffset - angleOffset;

                    steer = Math.signum(distance) * angleSteer * 0.02;

                    leftSpeed = speed - steer;
                    rightSpeed = speed + steer;

                    leftSpeed = Range.clip(leftSpeed, minSpeed, maxSpeed);
                    rightSpeed = Range.clip(rightSpeed, minSpeed, maxSpeed);

                    robot.frontLeftMotor.setPower(leftSpeed);
                    robot.frontRightMotor.setPower(rightSpeed);
                    robot.backLeftMotor.setPower(leftSpeed);
                    robot.backRightMotor.setPower(rightSpeed);

                    if (ct0 == 0) {
                        System.out.println("--BlueNear log-- error=" + String.format(Double.toString(error), "%5.2f")
                                + " leftspeed=" + String.format(Double.toString(leftSpeed), "%5.2f")
                                + " rightSpeed=" + String.format(Double.toString(rightSpeed), "%5.2f")
                                + " curAngle=" + angleOffset
                                + " targetAngle=" + targetAngleOffset);
                    }
                } else {

                    // distance on target. Need to keep 0 angle offset
                    // 3.0 is the expected front/back ultrasound sensor difference
                    targetAngleOffset = 0.0;

                    angleSteer = targetAngleOffset - angleOffset;

                    steer = Math.signum(distance) * angleSteer * 0.04;

                    leftSpeed = speed - steer;
                    rightSpeed = speed + steer;

                    leftSpeed = Range.clip(leftSpeed, minSpeed, maxSpeed);
                    rightSpeed = Range.clip(rightSpeed, minSpeed, maxSpeed);

                    robot.frontLeftMotor.setPower(leftSpeed);
                    robot.frontRightMotor.setPower(rightSpeed);
                    robot.backLeftMotor.setPower(leftSpeed);
                    robot.backRightMotor.setPower(rightSpeed);

                    if (ct0 == 0) {
                        System.out.println("--BlueNear log-- error=" + String.format(Double.toString(error), "%5.2f")
                                + " leftspeed=" + String.format(Double.toString(leftSpeed), "%5.2f")
                                + " rightSpeed=" + String.format(Double.toString(rightSpeed), "%5.2f")
                                + " curAngle=" + angleOffset
                                + " targetAngle=" + targetAngleOffset);
                    }
                }

                if (bLineDetection) {

                    lightlevel = robot.centerLightSensorLevelMax;

                    if (ct0 == 0) {
                        System.out.println("Ground Brightness:: " + robot.groundbrightnessAVG
                                + " Light Level:: " + lightlevel);
                    }

                    if (lightlevel > robot.groundbrightnessAVG * robot.CENTER_LIGHT_THRESH) {
                        System.out.println("Sigma2016 ---> Ground Brightness:: " + robot.groundbrightnessAVG
                                + " DETECTED Light Level:: " + lightlevel);
                        break;
                    }
                }
            }
        }

        // Turn off RUN_TO_POSITION
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (distance < 0) {
            if (robot.backLeftMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
                robot.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                robot.backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            }

            if (robot.backRightMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
                robot.backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                robot.backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            }
        }

        if (bLineDetection) {
            // turn off line light sensor reading
            sensorReadingThread.SetCenterLightSensorRead(false);
        }
    }

    // detect the color and push the blue button.
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

                // We are blue team
                robot.pusherR.setPosition(PUSHER_R_OUT);
                //wait servo to finish
                sleep(1300);

                // Retrieve the pusher
                robot.pusherR.setPosition(PUSHER_R_IN);
                //wait servo to finish
                sleep(1300);

                robot.pusherR.setPosition(PUSHER_STOP);

                System.out.println("--BlueNear log-- red light detected and blue button pushed. redCheck=" + redCheck + " blueCheck=" + blueCheck);
                break;
            }

            // blue color detected
            if (blueCheck > BLUE_TRESHOLD) {

                // We are the blue team
                robot.pusherL.setPosition(PUSHER_L_OUT);
                //wait servo to finish
                sleep(1300);

                // Retrieve the pusher
                robot.pusherL.setPosition(PUSHER_L_IN);
                //wait servo to finish
                sleep(1300);
                robot.pusherL.setPosition(PUSHER_STOP);

                System.out.println("--BlueNear log-- blue light detected and blue button pushed. blueCheck=" + blueCheck + " redCheck=" + redCheck);
                break;
            }

            sleep(10);
            idle();
        }
    }
}