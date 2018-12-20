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

import android.support.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@Autonomous(name="Depot Autonomous NO SAMPLING", group="Linear Opmode")
public class DepotAutoNoSampling extends LinearOpMode {

    // Declare OpMode members.
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AdCD63D/////AAAAGThVVvAi9UMXrbAu6IouvThcGKupmnakuvoWHZtIOH78mz1zQ+JAsVe7NqsffG4WpT1W2DvJQ8VsniObDD0N2W6y7WeavS8kseppMEdzy22UdVDXzvfPfoK/l62C3x0esCe7xeM8IOwZW8GtJX6cOalAR5HgYuS3VuN8eE/sPD9RmYwwRkhkGOntMOlWxc8yCIwTnn3nYBGEsOFEpz2+R+YboSIX2jWL1xs6Z7YqnA2rAAX489xbIoCsTWZEzQlPfbXk7frpTZpT7Nq3kh1PeGcRg536UTWGJ69fSRr8PIHJdycexY7uPhmfhEZBy3/pFOZ5lNhnqBuekll8PAhjftnvXeyRiWHugSEjVNUzdWhY";

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor armMotor;
    private DcMotor stringMotor;

    private Servo latch;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    //Encoder Math and Variables
    private static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 0.95 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double     DRIVE_SPEED             = 0.6;
    private static final double     TURN_SPEED              = 0.5;
    private static final double     ARM_EMPTY_SPEED         = 0.2;
    private static final double     STRING_EMPTY_SPEED      = 1;

    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        stringMotor = hardwareMap.get(DcMotor.class, "arm_motor2");
        latch = hardwareMap.get(Servo.class, "latch");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        composeTelemetry();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        boolean active = true;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && active && !gamepad1.b) {
            telemetry.addData("Active:", active);
            telemetry.update();

            //code goes here to make the robot do things

            sleep(100);
            detachFromLander();                 //Lower the Robot down from the lander
            sleep(100);
            lowerArmEmpty();
            sleep(100);

            centerPath();              //Chooses a path depending on where the gold mineral is.

            zeroPower();
            sleep(500);
            zeroPower();

            active = false;
        }
        //Stops the drive motors.
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        armMotor.setPower(0);
        stringMotor.setPower(0);
        latch.setPosition(0.5);

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        armMotor.setPower(0);
        stringMotor.setPower(0);
        latch.setPosition(0.5);

        stop();
    }

    //----------------------------------------------------------------------------------------------
    // Autonomous Paths
    //----------------------------------------------------------------------------------------------
    private void rightPath()
    {
        telemetry.addData("Running ", "Right Path!");
        telemetry.update();
        driveBackwardE( 4);
        sleep(100);
        turnLeftE(      40);
        sleep(100);
        driveBackwardE(24);
    }
    private void centerPath()
    {
        driveBackwardE( 45);
        sleep(500);
        driveForwardE(  20);
    }
    private void leftPath()
    {
        telemetry.addData("Running ", "Left Path!");
        telemetry.update();
        driveBackwardE( 4);
        sleep(100);
        turnRightE(     30);
        sleep(100);
        driveBackwardE( 24);
    }

    //----------------------------------------------------------------------------------------------
    // Driving Commands
    //----------------------------------------------------------------------------------------------
    private void driveForwardE(double distance)//Uses Encoders to Drive Forward
    {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            leftDrive.setPower(Math.abs(DRIVE_SPEED));
            rightDrive.setPower(Math.abs(DRIVE_SPEED));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < 30) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftDrive.getCurrentPosition(),
                        rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    private void driveBackwardE(double distance)//Uses Encoders to Drive Backward
    {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(-distance * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(-distance * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            leftDrive.setPower(Math.abs(DRIVE_SPEED));
            rightDrive.setPower(Math.abs(DRIVE_SPEED));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < 30) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftDrive.getCurrentPosition(),
                        rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    private void turnLeftE(int degrees)//Uses Encoders to Turn Left
    {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int)(-degrees * 3.75 /*Fancy Math*/);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(degrees * 3.75 /*Fancy Math*/);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            leftDrive.setPower(Math.abs(TURN_SPEED));
            rightDrive.setPower(Math.abs(TURN_SPEED));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < 30) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftDrive.getCurrentPosition(),
                        rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    private void turnRightE(int degrees)//Uses Encoders to Turn Right
    {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition()  +  (int)( degrees * 3.75);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(-degrees * 3.75);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            leftDrive.setPower(Math.abs(TURN_SPEED));
            rightDrive.setPower(Math.abs(TURN_SPEED));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < 30) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftDrive.getCurrentPosition(),
                        rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    //----------------------------------------------------------------------------------------------
    // Lander Attachment
    //----------------------------------------------------------------------------------------------
    private void detachFromLander()
    {
        telemetry.addData("Detacting from Lander: ", true);
        telemetry.update();
        lowerRobot();
        sleep(20);
        telemetry.addData("Driving Forward: ", 3);
        telemetry.update();
        driveForwardE(3);
        sleep(20);
        telemetry.addData("Latch ", "opening");
        telemetry.update();
        sleep(20);
        openLatch();
        sleep(100);
        driveForwardE(1);
        sleep(100);
        openLatch();
        sleep(100);
        openLatch();
        sleep(100);
        driveForwardE(2);
        sleep(100);
        openLatch();
        sleep(100);
        openLatch();
    }
    private void openLatch()
    {
        telemetry.addData("Setting Servo Power to ", "0");
        telemetry.update();
        latch.setPosition(0.5);
        sleep(25);

        telemetry.addData("Setting Servo Power to ", "100");
        telemetry.update();
        latch.setPosition(1);
        sleep(250);

        telemetry.addData("Setting Servo Power to ", "40");
        telemetry.update();
        latch.setPosition(0.7);
        sleep(250);

        telemetry.addData("Setting Servo Power to ", "100");
        telemetry.update();
        latch.setPosition(1);
        sleep(250);

        telemetry.addData("Setting Servo Power to ", "40");
        telemetry.update();
        latch.setPosition(0.7);
        sleep(250);

        telemetry.addData("Setting Servo Power to ", "0");
        telemetry.update();
        latch.setPosition(0.5);
        sleep(25);
    }
    private void lowerRobot()
    {
        telemetry.addData("Lowering Robot", true);
        telemetry.update();
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        sleep(100);
        while(angles.secondAngle > 5 || runtime.seconds() < 11 && opModeIsActive()) {
            armMotor.setPower(-0.3);
            stringMotor.setPower(0.75);
            telemetry.addData("Robot Angle: ", angles.secondAngle);
            telemetry.update();
        }
        armMotor.setPower(0);
        stringMotor.setPower(0);
        zeroPower();
    }
    private void lowerArmEmpty()
    {
        int newArmTarget;
        int newStringTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newArmTarget = 500;
            newStringTarget = 1000;
            armMotor.setTargetPosition(newArmTarget);
            stringMotor.setTargetPosition(newStringTarget);

            // Turn On RUN_TO_POSITION
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            stringMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            armMotor.setPower(Math.abs(ARM_EMPTY_SPEED));
            stringMotor.setPower(Math.abs(-STRING_EMPTY_SPEED));

            while (opModeIsActive() &&
                    (runtime.seconds() < 21) &&
                    (armMotor.isBusy() && stringMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newArmTarget, newStringTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        armMotor.getCurrentPosition(),
                        stringMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            armMotor.setPower(0);
            stringMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            stringMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    //----------------------------------------------------------------------------------------------
    // Other
    //----------------------------------------------------------------------------------------------
    private void zeroPower()
    {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        armMotor.setPower(0);
        stringMotor.setPower(0);
    }
    //----------------------------------------------------------------------------------------------
    // Gyroscope
    //----------------------------------------------------------------------------------------------
    private void composeTelemetry()
    {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }
    private String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    private String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}