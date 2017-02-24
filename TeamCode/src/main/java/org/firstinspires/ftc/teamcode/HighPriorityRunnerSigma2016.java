package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.HardwareSigma2016.POWER_ADJ_STEP;
import static org.firstinspires.ftc.teamcode.HardwareSigma2016.PUSHER_STOP;

public class HighPriorityRunnerSigma2016 extends Thread {

    // class variables
    HardwareSigma2016 robot = null;

    boolean bLineDetection = false;
    public boolean go = false;

    long lastOpModeLiveCheckTime = 0;

    // speed calculation variables
    int lastLeftEncoderPosition = 0;
    int lastRightEncoderPosition = 0;
    double lastAngle = 0.0;

    // methods
    HighPriorityRunnerSigma2016(HardwareSigma2016 ourHardware) {
        robot = ourHardware;

        this.setPriority(this.MAX_PRIORITY);
        this.go = true;
    }

    public int GetSpeed(long timeDelta) {
        int leftEncoderPosition = 0;
        int rightEncoderPosition = 0;
        double lastSpeed = 0.0;
        double angle = 0;
        double leftSpeed, rightSpeed;

        leftEncoderPosition = robot.LeftMotor.getCurrentPosition();
        rightEncoderPosition = robot.RightMotor.getCurrentPosition();

        leftSpeed = Math.abs(leftEncoderPosition - lastLeftEncoderPosition) / (double) timeDelta;
        rightSpeed = Math.abs(rightEncoderPosition - lastRightEncoderPosition) / (double) timeDelta;

        // calculate current speed -- unit inch per second
        robot.currentSpeed = Math.max(leftSpeed, rightSpeed) * 1000.0 / robot.COUNTS_PER_INCH;  // inch per second

//        System.out.println(leftEncoderPosition + "-" + lastLeftEncoderPosition + "=" + leftSpeed);
//        System.out.println(rightEncoderPosition + "-" + lastRightEncoderPosition + "=" + rightSpeed);
//        System.out.println("timeDelta=" + timeDelta);

        lastLeftEncoderPosition = leftEncoderPosition;
        lastRightEncoderPosition = rightEncoderPosition;

        angle = robot.gyro.getIntegratedZValue();
        robot.currentAngleSpeed = Math.abs(angle - lastAngle) * 1000 / timeDelta;  // degree per second
        lastAngle = angle;

        return (0);
    }

    public void run() {
        int lightlevelR, lightlevelG, lightlevelB;
        long curTime, timeInterval;
        long previousRunTime = 0, lastSpeedCheckTime = 0;
        int lightlevel = 0;

        // from Hitecnic color sensor web page
        // The Color Number calculated by the sensor is refreshed approximately 100 times per second.
        int expectedRunInterval = 10; // millisecond

        while (this.go) {

            // check if task runs on time
            curTime = System.currentTimeMillis();
            timeInterval = curTime - previousRunTime;
            if (timeInterval > expectedRunInterval * 2) {
                System.out.println("Sigma2016 -- high priority runner did not run for " + timeInterval + "ms");
            }

            // Get current speed information
            timeInterval = curTime - lastSpeedCheckTime;
            if (timeInterval > robot.SPEED_CHECK_INTERVAL) {
                GetSpeed(timeInterval);

                lastSpeedCheckTime = curTime;
            }

            previousRunTime = curTime;

            // line detection
            if (bLineDetection) {
                lightlevelB = robot.lineLightSensor.blue();
                lightlevelR = robot.lineLightSensor.red();
                lightlevelG = robot.lineLightSensor.green();

                lightlevel = lightlevelR + lightlevelG + lightlevelB;

                if (lightlevel > robot.maxLineBrightnessReading)
                {
                    robot.maxLineBrightnessReading = lightlevel;
                    System.out.println("Ground Brightness:: " + robot.groundbrightnessAVG
                            + " Light Level:: " + robot.maxLineBrightnessReading);
                }
            }

            if (robot.pusherRetractionStartTime > 0) {
                if (curTime - robot.pusherRetractionStartTime > 1600) {
                    robot.pusherR.setPosition(robot.PUSHER_STOP);
                    robot.pusherL.setPosition(robot.PUSHER_STOP);

                    robot.pusherRetractionStartTime = 0;
                }
            }

            // check if OP mode is still running. If OP mode is stopped, stop the high priority runner.
            if (robot.opModeLiveCheck == 1)
            {
                robot.opModeLiveCheck = 0;
                lastOpModeLiveCheckTime = curTime;
            }
            else
            {
                if (curTime - lastOpModeLiveCheckTime > 10000) // 10 seconds
                {
                    go = false;
                }
            }

            try {
                sleep(expectedRunInterval);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
