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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name="Super TeleOp", group="Iterative Opmode")
@Disabled
public class SuperTeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armMotor = null;
    private DcMotor armMotor2 = null;
    private Servo latch = null;

    private int switchy = 1;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        armMotor2 = hardwareMap.get(DcMotor.class, "arm_motor2");
        latch = hardwareMap.get(Servo.class, "latch");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

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
        gamepadOneStuff();
        gamepadTwoStuff();

        telemetry.addData("ArmMotor Position: ", armMotor.getCurrentPosition());
        telemetry.addData("StringMotor Position: ", armMotor2.getCurrentPosition());
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
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        armMotor.setPower(0);
        armMotor2.setPower(0);
    }
    private void gamepadOneStuff(){
        double leftPower = 0;
        double rightPower = 0;
        double leftY = gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;

        double rightY = gamepad1.right_stick_y;

        if (runtime.seconds() > 60) {
            switchy = 1;
        }
        else switchy = 1;
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

        leftDrive.setPower(leftPower * powermult * switchy);
        rightDrive.setPower(rightPower * powermult * switchy);

    }
    private void gamepadTwoStuff() {
        double armPower;
        double stringPower;

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

        latch.setPosition((-gamepad2.left_stick_x / 2) + 0.5);
        armMotor.setPower(armPower);
        armMotor2.setPower(stringPower);


        //down = xb
        //up = ay
    }

}
