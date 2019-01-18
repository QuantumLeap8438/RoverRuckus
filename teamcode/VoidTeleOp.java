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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

import static android.os.SystemClock.sleep;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Void TeleOp", group="Iterative Opmode")
@Disabled
public class VoidTeleOp extends OpMode
{
    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    private boolean xState = false;

    //private BNO055IMU imu;
    //private Orientation angles;
    //private Acceleration gravity;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        /*
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        */

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        /*
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        */

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        //composeTelemetry();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //------------------------------------------------------------------------------------------
        // GAMEPAD 1
        //------------------------------------------------------------------------------------------
        double leftPower = 0;
        double rightPower = 0;
        double drivePowerMult = .85;

        if (gamepad1.x) {
            xState = !xState;
            sleep(50);
        }

        if(xState) {
            leftPower = gamepad1.right_stick_y;
            rightPower = gamepad1.left_stick_y;
        }else {
            leftPower = gamepad1.left_stick_y;
            rightPower = gamepad1.right_stick_y;
        }


        if (gamepad1.dpad_up) {
            leftPower = -1;
            rightPower = -1;
        }
        else if (gamepad1.dpad_down) {
            leftPower = 1;
            rightPower = 1;
        }
        else if (gamepad1.dpad_left) {
            leftPower = 1;
            rightPower = -1;
        }
        else if (gamepad1.dpad_right) {
            leftPower = -1;
            rightPower = 1;
        }

        if(gamepad1.left_trigger > 0){
            drivePowerMult = .25;
        } else if(gamepad1.left_bumper){
            drivePowerMult = 1.5;
        }


        if (xState) {
            drivePowerMult = -drivePowerMult;
        }

        leftPower = -leftPower * drivePowerMult;
        rightPower = -rightPower * drivePowerMult;

        //------------------------------------------------------------------------------------------
        // GAMEPAD 2
        //------------------------------------------------------------------------------------------
        double armPower;
        double stringPower;
        double latchPos;
        double armPowerMult = 1;

        if(gamepad2.a) {
            armPower = 0.2;
        }
        else if(gamepad2.b) {
            armPower = -0.2;
        }
        else armPower = 0;

        if(gamepad2.x) {
            stringPower = 0.5;
        }
        else if(gamepad2.y) {
            stringPower = -0.5;
        }
        else stringPower = 0;

        if(gamepad2.right_trigger > 0.2) {
            stringPower = 0.5;
            armPower = -0.2;
        }
        else if(gamepad2.right_bumper) {
            stringPower = -0.5;
            armPower = 0.2;
        }

        if (gamepad2.right_stick_y != 0) {
            stringPower = gamepad2.right_stick_y;
            armPower = gamepad2.right_stick_y * 0.4;
        }

        if (gamepad2.left_bumper) {
            armPowerMult = 2;
        }
        else if (gamepad2.left_trigger > 0.2) {
            armPowerMult = 0.5;
        }

        latchPos = (-gamepad2.left_stick_x / 2) + 0.5;
        armPower = armPower * armPowerMult;
        stringPower = stringPower * armPowerMult;

        telemetry.addData("Left Drive Power: ", leftPower);
        telemetry.addData("Right Drive Power: ", rightPower);
        telemetry.addData("Reversed Mode is " , xState);
        telemetry.addData("Drive Controller is Currently ", "DRIVING");
        telemetry.addData("Gear Arm Power: ", armPower);
        telemetry.addData("String Arm Power: ", stringPower);
        telemetry.addData("Latch Power", (latchPos - 0.5) * 200);
        telemetry.addData("Currently ", "ARMING");

        //up = xb
        //down = ay

        telemetry.update();
        if (runtime.seconds() <= 119) {
            stop();
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        double leftPower = 0;
        double rightPower = 0;
        double armPower = 0;
        double stringPower = 0;
        double latchPos = 0;

        telemetry.addData("Left Drive Power: ", leftPower);
        telemetry.addData("Right Drive Power: ", rightPower);
        telemetry.addData("Gear Arm Power: ", armPower);
        telemetry.addData("String Arm Power: ", stringPower);
        telemetry.addData("Latch Power", (latchPos - 0.5) * 200);
        telemetry.addData("Currently ", "STOPPING THE ROBOT");

        telemetry.update();
    }
    /*
    private void gamepadOneStuff(){
        double leftPower = 0;
        double rightPower = 0;
        double leftX = gamepad1.left_stick_x;

        double rightY = gamepad1.right_stick_y;

        if(gamepad1.right_bumper && gamepad1.right_trigger <= 0.2) {
            if (rightY == 0) {
                rightPower = leftX * 1.5;
                leftPower = -leftX * 1.5;
            }
        }
        else if(gamepad1.right_bumper&& gamepad1.right_trigger >= 0.25) {
            leftPower = gamepad1.left_stick_y;
            rightPower = gamepad1.left_stick_y;
        }
        else {
            rightPower = gamepad1.left_stick_y;
            leftPower = gamepad1.right_stick_y;
        }

        double powermult = .85;

        if(gamepad1.left_trigger > 0){
            powermult = .25;
        } else if(gamepad1.left_bumper){
            powermult = 1;
        }

        leftPower = leftPower * powermult;
        rightPower = rightPower * powermult;
        telemetryDrive(leftPower, rightPower, "DRIVING");
    }
    private void gamepadTwoStuff() {
        double armPower;
        double stringPower;
        double latchPos;

        if(gamepad2.a) {
            armPower = 0.2;
        }
        else if(gamepad2.b) {
            armPower = -0.2;
        }
        else armPower = 0;

        if(gamepad2.x) {
            stringPower = 0.5;
        }
        else if(gamepad2.y) {
            stringPower = -0.5;
        }
        else stringPower = 0;

        if(gamepad2.left_bumper) {
            stringPower = -0.5;
            armPower = 0.2;
        }
        else if(gamepad2.right_bumper) {
            stringPower = 0.5;
            armPower = -0.2;
        }

        latchPos = (-gamepad2.left_stick_x / 2) + 0.5;

        telemetryArm(armPower, stringPower, latchPos, "ARMING");
        //down = xb
        //up = ay
    }
    */
    //----------------------------------------------------------------------------------------------
    private void tankDrive(double power, String mode) {
        double leftPower;
        double rightPower;

        double leftY = gamepad1.left_stick_y;
        double rightY = gamepad1.right_stick_y;

        leftPower = leftY * power;
        rightPower = rightY * power;

        //leftDrive.setPower(leftPower);
        //rightDrive.setPower(rightPower);
    }
    //----------------------------------------------------------------------------------------------
    /*
    private void composeTelemetry() {

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
    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    private String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    */
    //Custom Telemetry
    /*
    private void telemetryDrive(double leftPower, double rightPower, String currentAction) {
        telemetry.addData("Left Drive Power: ", leftPower);
        telemetry.addData("Right Drive Power: ", rightPower);
        telemetry.addData("Drive Controller is Currently ", currentAction);
    }
    private void telemetryArm(double armPower, double stringPower, double latchPos, String currentAction) {
        telemetry.addData("Gear Arm Power: ", armPower);
        telemetry.addData("String Arm Power: ", stringPower);
        telemetry.addData("Latch Power", (latchPos - 0.5) * 200);
        telemetry.addData("Currently ", currentAction);
    }
    private void telemetryUpdate() {
        telemetry.addData("Run Time is: ", runtime.seconds());
        telemetry.update();
        sleep(50);
    }
    */

}
