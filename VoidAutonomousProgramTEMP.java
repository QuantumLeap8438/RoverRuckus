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

@Autonomous(name="Void Autonomous TEMP", group="Linear Opmode")
public class VoidAutonomousProgramTEMP extends LinearOpMode {

    // Declare OpMode members.
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AdCD63D/////AAAAGThVVvAi9UMXrbAu6IouvThcGKupmnakuvoWHZtIOH78mz1zQ+JAsVe7NqsffG4WpT1W2DvJQ8VsniObDD0N2W6y7WeavS8kseppMEdzy22UdVDXzvfPfoK/l62C3x0esCe7xeM8IOwZW8GtJX6cOalAR5HgYuS3VuN8eE/sPD9RmYwwRkhkGOntMOlWxc8yCIwTnn3nYBGEsOFEpz2+R+YboSIX2jWL1xs6Z7YqnA2rAAX489xbIoCsTWZEzQlPfbXk7frpTZpT7Nq3kh1PeGcRg536UTWGJ69fSRr8PIHJdycexY7uPhmfhEZBy3/pFOZ5lNhnqBuekll8PAhjftnvXeyRiWHugSEjVNUzdWhY";

    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor leftDrive;
    //private DcMotor rightDrive;
    private DcMotor armMotor;
    private DcMotor stringMotor;

    private Servo latch;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    //Void Variables
    private int driveLeftPos = 0;
    private int driveRightPos = 0;
    private int armStringPos = 0;
    private int armGearPos = 0;

    private double leftPower = 0;
    private double rightPower = 0;
    private double gearPower = 0;
    private double stringPower = 0;
    private double latchPower = 0;

    private int leftTarget;
    private int rightTarget;

    private String currentAction = "Initalizing";

    private int bigStepState = 0;
    private int driveForwardState = 0;
    private int driveBackwardState = 0;
    private int turnLeftState = 0;
    private int turnRightState = 0;

    private int lowerRobotStepState = 0;

    //Encoder Math and Variables
    private static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 0.95 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double     DRIVE_SPEED             = 0.6;
    private static final double     TURN_SPEED              = 0.5;
    private static final boolean        THREE_MINERALS      = false;
    private static final boolean        TWO_MINERALS        = true;

    @Override
    public void runOpMode()
    {


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
        //leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        //rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        stringMotor = hardwareMap.get(DcMotor.class, "arm_motor2");
        latch = hardwareMap.get(Servo.class, "latch");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //leftDrive.setDirection(DcMotor.Direction.FORWARD);
        //rightDrive.setDirection(DcMotor.Direction.REVERSE);

        composeTelemetry();
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        boolean active = true;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && active && !gamepad1.b) {
            //Declaring Variables
            currentAction = "Starting the Program";
            //Taking Action
            if(gamepad1.a) {
                bigStepState = 1;
            }
            //Communicate and Evacuate
            if(bigStepState == 1) {
                telemetry.addData("Program is: ", "Complete");
                telemetry.update();
                active = false;
            } else {
                telemetry.addData("The Program is Currently: ", currentAction);
                telemetry.addLine();
                telemetry.addData("Left Drive Position:", driveLeftPos);
                telemetry.addData("Right Drive Position: ", driveRightPos);
                telemetry.addData("Gear Motor Position: ", armGearPos);
                telemetry.addData("String Motor Position: ", armStringPos);
                telemetry.addLine();
                telemetry.addData("Left Drive Power: ", leftPower);
                telemetry.addData("Right Drive Power: ", rightPower);
                telemetry.addData("Gear Motor Power: ", gearPower);
                telemetry.addData("String Motor Power: ", stringPower);
                telemetry.addData("Latch Servo Power: ", (latchPower - 0.5) * 200);
                telemetry.update();
            }
        }
        //Stops the motors and Servo.
        currentAction = "Stopping before Stopping";
        leftPower = 0;
        rightPower = 0;
        gearPower = 0;
        stringPower = 0;
        latchPower = 0.5;

        telemetry.addData("The Program is Currently: ", currentAction);
        telemetry.addLine();
        telemetry.addData("Left Drive Position:", driveLeftPos);
        telemetry.addData("Right Drive Position: ", driveRightPos);
        telemetry.addData("Gear Motor Position: ", armGearPos);
        telemetry.addData("String Motor Position: ", armStringPos);
        telemetry.addLine();
        telemetry.addData("Left Drive Power: ", leftPower);
        telemetry.addData("Right Drive Power: ", rightPower);
        telemetry.addData("Gear Motor Power: ", gearPower);
        telemetry.addData("String Motor Power: ", stringPower);
        telemetry.addData("Latch Servo Power: ", (latchPower - 0.5) * 200);
        telemetry.update();

        stop();
    }

    //----------------------------------------------------------------------------------------------
    // Autonomous Paths
    //----------------------------------------------------------------------------------------------
    private void rightDepotPath()
    {
        currentAction = "Running the Right Path(Depot Side)";
        driveBackwardE( 4);
        turnLeftE(      40);
        driveBackwardE(24);
    }
    private void centerDepotPath()
    {
        currentAction = "Running the Center Path(Depot Side)";
        driveBackwardE( 45);
        sleep(500);
        driveForwardE(  20);
    }
    private void leftDepotPath()
    {
        currentAction = "Running the Center Path(Depot Side)";
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
        if (opModeIsActive()) {
            if(driveForwardState == 0) {
                // Set Motor Positions
                leftTarget = driveLeftPos + (int)(distance * COUNTS_PER_INCH);
                rightTarget = driveRightPos + (int)(distance * COUNTS_PER_INCH);
                //Give the Motors Their Positions
                //leftDrive.setTargetPosition(leftTarget);
                //rightDrive.setTargetPosition(rightTarget);
            }
            // Turn On RUN_TO_POSITION
            //leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                                                                                    /*BREAK*/
            //Set Motor Power
            leftPower = DRIVE_SPEED;
            rightPower = DRIVE_SPEED;

            if (driveLeftPos > leftTarget && driveRightPos > rightTarget) {
                leftPower = 0;
                rightPower = 0;
                driveForwardState = 2;
                // Stop all motion;
                //leftDrive.setPower(leftPower);
                //rightDrive.setPower(rightPower);

                // Turn off RUN_TO_POSITION
                //leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //  sleep(250);   // optional pause after each move
            }

            if(driveForwardState == 0) {
                driveForwardState = 1;
            }
        }
    }
    private void driveBackwardE(double distance)//Uses Encoders to Drive Backward
    {
        if (opModeIsActive()) {
            if(driveBackwardState == 0) {
                // Set Motor Positions
                leftTarget = driveLeftPos + (int)(-distance * COUNTS_PER_INCH);
                rightTarget = driveRightPos + (int)(-distance * COUNTS_PER_INCH);
                //Give the Motors Their Positions
                //leftDrive.setTargetPosition(leftTarget);
                //rightDrive.setTargetPosition(rightTarget);
            }
            // Turn On RUN_TO_POSITION
            //leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            /*BREAK*/
            //Set Motor Power
            leftPower = DRIVE_SPEED;
            rightPower = DRIVE_SPEED;

            if (driveLeftPos < leftTarget && driveRightPos < rightTarget) {
                leftPower = 0;
                rightPower = 0;
                driveBackwardState = 2;
                // Stop all motion;
                //leftDrive.setPower(leftPower);
                //rightDrive.setPower(rightPower);

                // Turn off RUN_TO_POSITION
                //leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //  sleep(250);   // optional pause after each move
            }

            if(driveBackwardState == 0) {
                driveBackwardState = 1;
            }
        }
    }
    private void turnLeftE(int degrees)//Uses Encoders to Turn Left
    {
        if (opModeIsActive()) {
            if(turnLeftState == 0) {
                // Set Motor Positions
                leftTarget = driveLeftPos + (int)(-degrees * COUNTS_PER_INCH);
                rightTarget = driveRightPos + (int)(degrees * COUNTS_PER_INCH);
                //Give the Motors Their Positions
                //leftDrive.setTargetPosition(leftTarget);
                //rightDrive.setTargetPosition(rightTarget);
            }
            // Turn On RUN_TO_POSITION
            //leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            /*BREAK*/
            //Set Motor Power
            leftPower = TURN_SPEED;
            rightPower = TURN_SPEED;

            if (driveLeftPos < leftTarget || driveRightPos > rightTarget) {
                leftPower = 0;
                rightPower = 0;
                turnLeftState = 2;
                // Stop all motion;
                //leftDrive.setPower(leftPower);
                //rightDrive.setPower(rightPower);

                // Turn off RUN_TO_POSITION
                //leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //  sleep(250);   // optional pause after each move
            }

            if(turnLeftState == 0) {
                turnLeftState = 1;
            }
        }
    }
    private void turnRightE(int degrees)//Uses Encoders to Turn Right
    {
        if (opModeIsActive()) {
            if(turnRightState == 0) {
                // Set Motor Positions
                leftTarget = driveLeftPos + (int)(degrees * COUNTS_PER_INCH);
                rightTarget = driveRightPos + (int)(-degrees * COUNTS_PER_INCH);
                //Give the Motors Their Positions
                //leftDrive.setTargetPosition(leftTarget);
                //rightDrive.setTargetPosition(rightTarget);
            }
            // Turn On RUN_TO_POSITION
            //leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                                                                                    /*BREAK*/
            //Set Motor Power
            leftPower = TURN_SPEED;
            rightPower = TURN_SPEED;

            if (driveLeftPos > leftTarget || driveRightPos < rightTarget) {
                leftPower = 0;
                rightPower = 0;
                turnRightState = 2;
                // Stop all motion;
                //leftDrive.setPower(leftPower);
                //rightDrive.setPower(rightPower);

                // Turn off RUN_TO_POSITION
                //leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //  sleep(250);   // optional pause after each move
            }

            if(turnRightState == 0) {
                turnRightState = 1;
            }
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
        currentAction = "Lowering the Robot";
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        sleep(100);
        if(angles.secondAngle > 5 || runtime.seconds() < 11 && lowerRobotStepState < 2) {
            gearPower = 0.3;
            stringPower = 0.75;
            telemetry.addData("Robot Angle: ", angles.secondAngle);
            telemetry.update();
        }
        gearPower = 0;
        stringPower = 0;
        zeroPower();
    }
    private void lowerArmEmpty(int timeout) {
        telemetry.addData("Retracting ", "Arm!");
        telemetry.update();
        boolean go = true;
        while (runtime.seconds() <= timeout && opModeIsActive()) {
            armMotor.setPower(0.12);
            stringMotor.setPower(-0.99);
        }
        armMotor.setPower(0);
        stringMotor.setPower(0);
    }
    //----------------------------------------------------------------------------------------------
    // Mineral Recognition and Decisions
    //----------------------------------------------------------------------------------------------
    private void pathOfGold(@NonNull String goldPos) //Path Decider
    {
        //Uses goldPos to decide which path to take.
        if(goldPos.equals("Left")) {
            leftDepotPath();
        }
        else if(goldPos.equals("Center")) {
            centerDepotPath();
        }
        else if(goldPos.equals("Right")) {
            rightDepotPath();
        }
    }
    private void initTfod() //Initialize Tensor Flow
    {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    private void initVuforia() //Initialize Vuforia
    {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }
    private String tensorFlowDetection() //Tensor Flow Mineral Detection
    {
        String goldPos = null;
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) initTfod();
        else telemetry.addData("Sorry!", "This device is not compatible with TFOD");


        if (opModeIsActive()) {
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive() && goldPos == null && runtime.seconds() <= 8) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3 && THREE_MINERALS) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    goldPos = "Left";
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    goldPos = "Right";
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    goldPos = "Center";
                                }
                            }
                        } else if (updatedRecognitions.size() == 2 && TWO_MINERALS) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1) {
                                if (goldMineralX < silverMineral1X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    goldPos = "Left";
                                } else if (goldMineralX > silverMineral1X) {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    goldPos = "Center";
                                }
                            } else if (silverMineral1X != -1 && silverMineral2X != -1) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                goldPos = "Right";
                            }

                            telemetry.update();
                        }
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        if (goldPos == null) {
            goldPos = "Center";
        }
        return goldPos;
    }

    //----------------------------------------------------------------------------------------------
    // Other
    //----------------------------------------------------------------------------------------------
    private void zeroPower()
    {
        leftPower = 0;
        rightPower = 0;
        gearPower = 0;
        stringPower = 0;
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
}