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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

import static android.os.SystemClock.sleep;

@Autonomous(name="IterativeAutonomous NO SAMPLING", group="Iterative Opmode")
@Disabled
public class SuperAutonomousNoSampling extends OpMode
{
    private boolean detect = false;
    private boolean startPlace = true; //Depot == true, Crater == false
    //--------------------------------------------------------------------------------------------\\
    //============================================================================================\\
    // DECLARING OPMODE MEMBERS \\
    //============================================================================================\\
    //--------------------------------------------------------------------------------------------\\

    //Hardware
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor armMotor;
    private DcMotor stringMotor;
    private Servo latch;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    //Position and Speed Variables
    private String goldPos;
    private boolean viewSide = true; //Left == false, Right == true


    private int driveLeftPos, driveRightPos, armStringPos, armGearPos = 0;

    private double leftPower, rightPower, gearPower, stringPower = 0;
    private double latchPower = 0.5;

    private int leftTarget, rightTarget, gearTarget, stringTarget;

    private boolean leftEncoderDrive, rightEncoderDrive, gearEncoderDrive, stringEncoderDrive = false;

    private String currentAction = "Declaring";

    //--------------------------------------------------------------------------------------------\\
    //============================================================================================\\
    //                       PATH VARIABLES FOR DISTANCE AND STATE TRACKING                       \\
    //============================================================================================\\
    //--------------------------------------------------------------------------------------------\\

    // Drive Distances
    private double driveDistance, turnDistance;
    // Path Distance Variables
    //Depot
    private final int rightDepotPathFirstDistance = 4;
    private final int rightDepotPathSecondDistance = 40;
    private final int rightDepotPathThirdDistance = 24;

    private final int centerDepotPathFirstDistance = 45;
    private final int centerDepotPathSecondDistance = 20;

    private final int leftDepotPathFirstDistance = 4;
    private final int leftDepotPathSecondDistance = 30;
    private final int leftDepotPathThirdDistance = 24;

    //Crater
    private final int rightCraterPathFirstDistance = 4;
    private final int rightCraterPathSecondDistance = 40;
    private final int rightCraterPathThirdDistance = 24;

    private final int centerCraterPathFirstDistance = 47;

    private final int leftCraterPathFirstDistance = 4;
    private final int leftCraterPathSecondDistance = 30;
    private final int leftCraterPathThirdDistance = 24;

    //Detach
    private final int detachFirstDistance = 3;
    private final int detachSecondDistance = 1;
    private final int detachThirdDistance = 2;

    //State Variables
    private final int bigStepBreak = 4;                 private int bigStepState = 0;
    private boolean initializeState = false;
    private boolean runState = false;
    private boolean stopState = false;
    private boolean paused = false;

    private final int pathDoerBreak = 1;                private int pathDoerState = pathDoerBreak;
    private final int rightDepotPathBreak = 4;          private int rightDepotPathState = rightDepotPathBreak;
    private final int centerDepotPathBreak = 3;         private int centerDepotPathState = centerDepotPathBreak;
    private final int leftDepotPathBreak = 4;           private int leftDepotPathState = leftDepotPathBreak;

    private final int rightCraterPathBreak = 4;         private int rightCraterPathState = rightCraterPathBreak;
    private final int centerCraterPathBreak = 3;        private int centerCraterPathState = centerCraterPathBreak;
    private final int leftCraterPathBreak = 4;          private int leftCraterPathState = leftCraterPathBreak;

    private final int driveForwardBreak = 2;            private int driveForwardState = driveForwardBreak;
    private final int driveBackwardBreak = 2;           private int driveBackwardState = driveBackwardBreak;
    private final int turnLeftBreak = 2;                private int turnLeftState = turnLeftBreak;
    private final int turnRightBreak = 2;               private int turnRightState = turnRightBreak;

    private final int lowerRobotBreak = 3;              private int lowerRobotState = lowerRobotBreak;
    private final int openLatchBreak = 11;              private int openLatchState = openLatchBreak;
    private int resetArmBreak = 2;                      private int resetArmState = resetArmBreak;
    private int detachBreak = 8;                        private int detachState = detachBreak;

    private int mineralDetectionBreak = 3;              private int mineralDetectionState = mineralDetectionBreak;

    //Encoder Math and Variables
    private static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // REV Core Hex Motor
    private static final double     DRIVE_GEAR_REDUCTION    = 0.95 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    //Default Motor Speeds
    private static final double     DRIVE_SPEED             = 0.6;
    private static final double     TURN_SPEED              = 0.5;
    private static final double     ARM_SPEED               = 0.3;
    private static final double     STRING_SPEED            = 0.6;

    //--------------------------------------------------------------------------------------------\\
    //============================================================================================\\
    //                                       OPMODE METHODS                                       \\
    //============================================================================================\\
    //--------------------------------------------------------------------------------------------\\

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        initializeState = true;
        // IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        stringMotor = hardwareMap.get(DcMotor.class, "arm_motor2");
        latch = hardwareMap.get(Servo.class, "latch");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Motor Direction
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        stringMotor.setDirection(DcMotor.Direction.FORWARD);

        giveTelemetry();
        composeTelemetry();

        // Tell the driver that initialization is complete.
        currentAction = "Initialized";
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        //Telemetry
        giveTelemetry();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        runState = true;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Send Telemetry to Driver
        giveTelemetry();

        //Run Methods According to Their State Values
        blockRunner();

        if (paused) {
            leftPower = 0;
            rightPower = 0;
            gearPower = 0;
            stringPower = 0;
            latchPower = 0.5;
        }
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        armMotor.setPower(gearPower);
        stringMotor.setPower(stringPower);
        latch.setPosition(latchPower);

        //Program Pausing
        if (gamepad1.a) {
            paused = true;
        }
        else if (gamepad1.b) {
            paused = false;
        }

        if (gamepad1.x) {
            stopState = true;
        }

        //Emergency Stop Caller
        if (stopState) {
            zeroPower();
            stop();
        }

        // Send Telemetry to Driver
        giveTelemetry();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {
        currentAction = "Stopping";
        leftPower = 0;
        rightPower = 0;
        gearPower = 0;
        stringPower = 0;
        latchPower = 0.5;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        armMotor.setPower(gearPower);
        stringMotor.setPower(stringPower);
        latch.setPosition(latchPower);

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        armMotor.setPower(0);
        stringMotor.setPower(0);
        latch.setPosition(0.5);

        giveTelemetry();
        sleep(1000);
    }
    //--------------------------------------------------------------------------------------------\\
    //============================================================================================\\
    //                                       BLOCK METHODS                                        \\
    //============================================================================================\\
    //--------------------------------------------------------------------------------------------\\
    private void giveTelemetry()
    {
        telemetry.addData("The Robot is Currently: ", currentAction);
        telemetry.addData("The Gold Mineral is in the ", goldPos + " Position");
        telemetry.addData("Paused: ", paused);
        telemetry.addLine();
        telemetry.addData("Left Motor Power: ", leftPower);
        telemetry.addData("Right Motor Power: ", rightPower);
        telemetry.addData("Gear Motor Power: ", gearPower);
        telemetry.addData("String Power: ", stringPower);
        telemetry.addData("Latch Power: ", latchPower);
        telemetry.addLine();
        telemetry.addData("Left Motor Position: ", driveLeftPos);
        telemetry.addData("Right Motor Position: ", driveRightPos);
        telemetry.addData("Gear Motor Position: ", armGearPos);
        telemetry.addData("String Motor Position: ", armStringPos);
        telemetry.addLine();
        telemetry.addData("Left Motor Target: ", leftTarget);
        telemetry.addData("Right Motor Target: ", rightTarget);
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addData("Big Step State: ", bigStepState + " / " + bigStepBreak);
        telemetry.addData("Path Chooser State: ", pathDoerState + " / " + pathDoerBreak);
        telemetry.addLine();
        telemetry.addData("Right Depot Path State: ", rightDepotPathState + " / " + rightDepotPathBreak);
        telemetry.addData("Center Depot Path State: ", centerDepotPathState + " / " + centerDepotPathBreak);
        telemetry.addData("Left Depot Path State: ", leftDepotPathState + " / " + leftDepotPathBreak);
        telemetry.addData("Right Crater Path State: ", rightCraterPathState + " / " + rightCraterPathBreak);
        telemetry.addData("Center Crater Path State: ", centerCraterPathState + " / " + centerDepotPathBreak);
        telemetry.addData("Left Crater Path State: ", leftCraterPathState + " / " + leftCraterPathBreak);
        telemetry.addLine();
        telemetry.addData("Drive Forward State: ", driveForwardState + " / " + driveForwardBreak);
        telemetry.addData("Drive Backward State: ", driveBackwardState + " / " + driveBackwardBreak);
        telemetry.addData("Turn Left State: ", turnLeftState + " / " + turnLeftBreak);
        telemetry.addData("Turn Right State: ", turnRightState + " / " + turnRightBreak);
        telemetry.addLine();
        telemetry.addData("Lower Robot State: ", lowerRobotState + " / " + lowerRobotBreak);
        telemetry.addData("Open Latch State: ", openLatchState + " / " + openLatchBreak);
        telemetry.addData("Reset Arm State: ", resetArmState + " / " + resetArmBreak);
        telemetry.addData("Detach from Lander State: ", detachState + " / " + detachBreak);
        telemetry.addLine();
        telemetry.addData("Image Detection State: ", mineralDetectionState + " / " + mineralDetectionBreak);
        telemetry.addData("View Side: ", viewSide);
        telemetry.addLine();

        if(runState) {
            composeTelemetry();
        }

        telemetry.update();
    }
    private void blockRunner()
    {
        //Big Stepper
        bigStepper();

        //Path Methods
        pathOfGold();

        leftDepotPath();
        rightDepotPath();
        centerDepotPath();

        //Driving Methods
        driveForwardE();
        driveBackwardE();
        turnLeftE();
        turnRightE();

        //Lander Methods
        lowerRobot();
        openLatch();
        resetArm();
        detachFromLander();
    }
    private void bigStepper()
    {
        //Image Regognition
        if(bigStepState == 0) {
            if (detect) {
                mineralDetectionState = 0;
            }
            bigStepState = 1;
        }
        //Lander Detachment
        if(bigStepState == 1 && (mineralDetectionState == mineralDetectionBreak || !detect)) {
            detachState = 0;
            bigStepState = 2;
        }
        //Driving
        if (bigStepState == 2 && detachState == detachBreak) {
            pathDoerState = 0;
            bigStepState = 3;
        }
        if (bigStepState == 3 && pathDoerState == pathDoerBreak) {
            bigStepState = 4;
        }

    }
    //----------------------------------------------------------------------------------------------
    // AUTONOMOUS PATHS
    //----------------------------------------------------------------------------------------------
    private void rightDepotPath() //Drive Backward 4", Turn Left 40 degrees, Drive Backward 24"
    {
        if (rightDepotPathState == 0) {
            driveDistance = rightDepotPathFirstDistance;
            driveBackwardState = 0;
            rightDepotPathState = 1;
        }
        if (rightDepotPathState == 1 && driveBackwardState == driveBackwardBreak) {
            turnDistance = rightDepotPathSecondDistance;
            turnLeftState = 0;
            rightDepotPathState = 2;
        }
        if (rightDepotPathState == 2 && turnLeftState == turnLeftBreak) {
            driveDistance = rightDepotPathThirdDistance;
            driveBackwardState = 0;
            rightDepotPathState = 3;
        }
        if (rightDepotPathState == 3 && driveBackwardState == driveBackwardBreak) {
            rightDepotPathState = 4;
        }
    }
    private void centerDepotPath() //Drive Backward 45", Drive Forward 20"
    {
        if (centerDepotPathState == 0) {
            driveDistance = centerDepotPathFirstDistance;
            driveBackwardState = 0;
            centerDepotPathState = 1;
        }
        if (centerDepotPathState == 1 && driveBackwardState == 2) {
            driveDistance = centerDepotPathSecondDistance;
            driveForwardState = 0;
            centerDepotPathState = 2;
        }
        if (centerDepotPathState == 2 && driveForwardState == driveForwardBreak) {
            centerDepotPathState = 3;
        }
    }
    private void leftDepotPath() //Drive Backward 4", Turn Right 30 degrees, Drive Backward 24"
    {
        if (leftDepotPathState == 0) {
            driveDistance = leftDepotPathFirstDistance;
            driveBackwardState = 0;
            leftDepotPathState = 1;
        }
        if (leftDepotPathState == 1 && driveBackwardState == driveBackwardBreak) {
            turnDistance = leftDepotPathSecondDistance;
            turnRightState = 0;
            leftDepotPathState = 2;
        }
        if (leftDepotPathState == 2 && turnRightState == turnRightBreak) {
            driveDistance = leftDepotPathThirdDistance;
            driveBackwardState = 0;
            leftDepotPathState = 3;
        }
        if (leftDepotPathState == 3 && driveBackwardState == driveBackwardBreak) {
            leftDepotPathState = 4;
        }
    }
    private void rightCraterPath() //Drive Backward 4", Turn Left 40 degrees, Drive Backward 24"
    {
        if (rightCraterPathState == 0) {
            driveDistance = rightCraterPathFirstDistance;
            driveBackwardState = 0;
            rightCraterPathState = 1;
        }
        if (rightCraterPathState == 1 && driveBackwardState == driveBackwardBreak) {
            turnDistance = rightCraterPathSecondDistance;
            turnLeftState = 0;
            rightCraterPathState = 2;
        }
        if (rightCraterPathState == 2 && turnLeftState == turnLeftBreak) {
            driveDistance = rightCraterPathThirdDistance;
            driveBackwardState = 0;
            rightCraterPathState = 3;
        }
        if (rightCraterPathState == 3 && driveBackwardState == driveBackwardBreak) {
            rightCraterPathState = 4;
        }
    }
    private void centerCraterPath() //Drive Backward 47"
    {
        if (centerCraterPathState == 0) {
            driveDistance = centerCraterPathFirstDistance;
            driveBackwardState = 0;
            centerCraterPathState = 1;
        }
        if (centerCraterPathState == 1 && driveBackwardState == 2) {
            centerCraterPathState = 2;
        }
    }
    private void leftCraterPath() //Drive Backward 4", Turn Right 30 degrees, Drive Backward 24"
    {
        if (leftCraterPathState == 0) {
            driveDistance = leftCraterPathFirstDistance;
            driveBackwardState = 0;
            leftCraterPathState = 1;
        }
        if (leftCraterPathState == 1 && driveBackwardState == driveBackwardBreak) {
            turnDistance = leftCraterPathSecondDistance;
            turnRightState = 0;
            leftCraterPathState = 2;
        }
        if (leftCraterPathState == 2 && turnRightState == turnRightBreak) {
            driveDistance = leftCraterPathThirdDistance;
            driveBackwardState = 0;
            leftCraterPathState = 3;
        }
        if (leftCraterPathState == 3 && driveBackwardState == driveBackwardBreak) {
            leftCraterPathState = 4;
        }
    }

    //----------------------------------------------------------------------------------------------
    // Driving Commands
    //----------------------------------------------------------------------------------------------
    private void driveForwardE()//Uses Encoders to Drive Forward
    {
        if(driveForwardState == 0) {
            // Set Motor Positions
            leftTarget = leftDrive.getCurrentPosition() + (int)(driveDistance * COUNTS_PER_INCH);
            rightTarget = leftDrive.getCurrentPosition() + (int)(driveDistance * COUNTS_PER_INCH);
            //Give the Motors Their Positions
            leftDrive.setTargetPosition(leftTarget);
            rightDrive.setTargetPosition(rightTarget);
            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftEncoderDrive = true;
            rightEncoderDrive = true;
            driveForwardState = 1;
        }
        if (driveForwardState == 1) {
            //Set Motor Power
            leftPower = DRIVE_SPEED;
            rightPower = DRIVE_SPEED;

            driveLeftPos = leftDrive.getCurrentPosition();
            driveRightPos = rightDrive.getCurrentPosition();

            if (driveLeftPos >= leftTarget && driveRightPos >= rightTarget) {
                leftPower = 0;
                rightPower = 0;
                // Turn off RUN_TO_POSITION
                leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftEncoderDrive = false;
                rightEncoderDrive = false;

                driveForwardState = 2;
            }
        }
    }
    private void driveBackwardE()//Uses Encoders to Drive Backward
    {
        if(driveBackwardState == 0) {
            // Set Motor Positions
            leftTarget = leftDrive.getCurrentPosition() + (int)(-driveDistance * COUNTS_PER_INCH);
            rightTarget = rightDrive.getCurrentPosition() + (int)(-driveDistance * COUNTS_PER_INCH);
            //Give the Motors Their Positions
            leftDrive.setTargetPosition(leftTarget);
            rightDrive.setTargetPosition(rightTarget);
            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftEncoderDrive = true;
            rightEncoderDrive = true;
            driveBackwardState = 1;
        }
        if (driveBackwardState == 1) {
            //Set Motor Power
            leftPower = DRIVE_SPEED;
            rightPower = DRIVE_SPEED;

            driveLeftPos = leftDrive.getCurrentPosition();
            driveRightPos = rightDrive.getCurrentPosition();

            if (driveLeftPos <= leftTarget && driveRightPos <= rightTarget) {
                leftPower = 0;
                rightPower = 0;

                // Turn off RUN_TO_POSITION
                leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftEncoderDrive = false;
                rightEncoderDrive = false;

                driveBackwardState = 2;
            }
        }
    }
    private void turnLeftE()//Uses Encoders to Turn Left
    {
        if(turnLeftState == 0) {
            // Set Motor Positions
            leftTarget = leftDrive.getCurrentPosition() + (int)(-turnDistance * COUNTS_PER_INCH);
            rightTarget = rightDrive.getCurrentPosition() + (int)(turnDistance * COUNTS_PER_INCH);
            //Give the Motors Their Positions
            leftDrive.setTargetPosition(leftTarget);
            rightDrive.setTargetPosition(rightTarget);
            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftEncoderDrive = true;
            rightEncoderDrive = true;
            turnLeftState = 1;
        }
        if (turnLeftState == 1) {
            //Set Motor Power
            leftPower = TURN_SPEED;
            rightPower = TURN_SPEED;

            driveLeftPos = leftDrive.getCurrentPosition();
            driveRightPos = rightDrive.getCurrentPosition();

            if (driveLeftPos <= leftTarget || driveRightPos >= rightTarget) {
                leftPower = 0;
                rightPower = 0;


                // Turn off RUN_TO_POSITION
                leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftEncoderDrive = false;
                rightEncoderDrive = false;

                turnLeftState = 2;
            }
        }
    }
    private void turnRightE()//Uses Encoders to Turn Right
    {
        if(turnRightState == 0) {
            // Set Motor Positions
            leftTarget = leftDrive.getCurrentPosition() + (int)(turnDistance * COUNTS_PER_INCH);
            rightTarget = rightDrive.getCurrentPosition() + (int)(-turnDistance * COUNTS_PER_INCH);
            //Give the Motors Their Positions
            leftDrive.setTargetPosition(leftTarget);
            rightDrive.setTargetPosition(rightTarget);
            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftEncoderDrive = true;
            rightEncoderDrive = true;
            turnRightState = 1;
        }
        if (turnRightState == 1) {
            //Set Motor Power
            leftPower = TURN_SPEED;
            rightPower = TURN_SPEED;

            driveLeftPos = leftDrive.getCurrentPosition();
            driveRightPos = rightDrive.getCurrentPosition();

            if (driveLeftPos >= leftTarget || driveRightPos <= rightTarget) {
                leftPower = 0;
                rightPower = 0;

                // Turn off RUN_TO_POSITION
                leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftEncoderDrive = false;
                rightEncoderDrive = false;

                turnRightState = 2;
            }
        }
    }

    //----------------------------------------------------------------------------------------------
    // LANDER DETACHMENT
    //----------------------------------------------------------------------------------------------
    private void lowerRobot()
    {
        if(lowerRobotState == 0){
            currentAction = "Lowering the Robot";
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gearEncoderDrive = false;
            stringEncoderDrive = false;
            lowerRobotState = 1;
        }
        if(angles.secondAngle > 5 && lowerRobotState == 1 && runtime.seconds() < 5) {
            gearPower = 0.3;
            stringPower = 0.75;
        }
        else if (lowerRobotState != 3) lowerRobotState = 2;

        if(lowerRobotState == 2) {
            gearPower = 0;
            stringPower = 0;
            zeroPower();
            lowerRobotState = 3;
        }
    }
    private void openLatch()
    {
        if (openLatchState <= 10) {
            if (openLatchState % 2 == 0) {
                latchPower = 0.7;
                sleep(20);
            }
            if (openLatchState % 2 == 1) {
                latchPower = 0.9;
                sleep(20);
            }
            openLatchState++;
        }
    }
    private void resetArm()//Uses Encoders to Reset the Arm
    {
        if(resetArmState == 0) {
            // Set Motor Positions
            gearTarget = 0;
            stringTarget = 0;
            //Give the Motors Their Positions
            armMotor.setTargetPosition(gearTarget);
            stringMotor.setTargetPosition(stringTarget);
            // Turn On RUN_TO_POSITION
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            stringMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            gearEncoderDrive = true;
            stringEncoderDrive = true;
            resetArmState = 1;
        }
        if (resetArmState == 1) {
            gearPower = ARM_SPEED;
            rightPower = STRING_SPEED * 2;

            armGearPos = armMotor.getCurrentPosition();
            armStringPos = stringMotor.getCurrentPosition();

            if (gearTarget <= armGearPos && stringTarget <= armStringPos && runtime.seconds() < 14) {
                zeroPower();
                // Turn off RUN_TO_POSITION
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                stringMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                gearEncoderDrive = false;
                stringEncoderDrive = false;
                resetArmState = 2;
            }
        }
    }
    private void detachFromLander() // Lower Robot, Drive Forward 3, Open Latch, Drive Forward 1, Open Latch, Drive Backward 2, Reset Arm
    {
        if (detachState == 0) {
            lowerRobotState = 0;
            detachState = 1;
        }
        if (detachState == 1 && lowerRobotState == lowerRobotBreak) {
            driveDistance = detachFirstDistance;
            driveForwardState = 0;
            detachState = 2;
        }
        if (detachState == 2 && driveForwardState == driveForwardBreak) {
            openLatchState = 0;
            detachState = 3;
        }
        if (detachState == 3 && openLatchState == openLatchBreak) {
            driveDistance = detachSecondDistance;
            driveForwardState = 0;
            detachState = 4;
        }
        if (detachState == 4 && driveForwardState == driveForwardBreak) {
            openLatchState = 0;
            detachState = 5;
        }
        if (detachState == 5 && openLatchState == openLatchBreak) {
            driveDistance = detachThirdDistance;
            driveBackwardState = 0;
            detachState = 6;
        }
        if (detachState == 6 && driveBackwardState == driveBackwardBreak) {
            resetArmState = 0;
            detachState = 7;
        }
        if (detachState == 7 && resetArmState == resetArmBreak) {
            detachState = 8;
        }
    }

    //----------------------------------------------------------------------------------------------
    // MINERAL RECOGNITION AND DECISIONS
    //----------------------------------------------------------------------------------------------
    private void pathOfGold()
    {
        if (startPlace) {
            if (pathDoerState == 0) {
                if (detect) {
                    if (goldPos.equals("Right")) {
                        rightDepotPathState = 0;
                        pathDoerState = 1;
                    }
                    if (goldPos.equals("Left")) {
                        leftDepotPathState = 0;
                        pathDoerState = 1;
                    }
                    if (!goldPos.equals("Left")  &&  !goldPos.equals("Right") ) {
                        centerDepotPathState = 0;
                        pathDoerState = 1;
                    }
                }
                else {
                    centerDepotPathState = 0;
                    pathDoerState = 1;
                }
            }
        } else {
            if (pathDoerState == 0) {
                if (detect) {
                    if (goldPos.equals("Right")) {
                        rightCraterPathState = 0;
                        pathDoerState = 1;
                    }
                    if (goldPos.equals("Left")) {
                        leftCraterPathState = 0;
                        pathDoerState = 1;
                    }
                    if (!goldPos.equals("Left")  &&  !goldPos.equals("Right") ) {
                        centerCraterPathState = 0;
                        pathDoerState = 1;
                    }
                }
                else {
                    centerCraterPathState = 0;
                    pathDoerState = 1;
                }
            }
        }

    }

    //----------------------------------------------------------------------------------------------
    // GYROSCOPE
    //----------------------------------------------------------------------------------------------

    @NonNull
    private String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    @NonNull
    private String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    //--------------------------------------------------------------------------------------------\\
    //                                           OTHER                                            \\
    //--------------------------------------------------------------------------------------------\\
    private void zeroPower()
    {
        leftPower = 0;
        rightPower = 0;
        gearPower = 0;
        stringPower = 0;
        latchPower = 0;
    }
    void composeTelemetry() {

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

    //--------------------------------------------------------------------------------------------\\
    //============================================================================================\\
    //                                     END OF CODE SPACER                                     \\
    //============================================================================================\\
    //--------------------------------------------------------------------------------------------\\

    //----------------------------------------------------------------------------------------------
    // THE VOID
    //----------------------------------------------------------------------------------------------
    /*


     */
}