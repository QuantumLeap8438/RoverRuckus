package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.TetrixMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="UltimateComboTeleOp(17-18)", group="Iterative Opmode")
@Disabled
public class UltimateComboTeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    //private DcMotor flag = null;

    private DcMotor glypherArmTilt = null;
    private DcMotor glypherArmYax = null;
    private DcMotor glypherPinch = null;


    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        // Initialize the hardware variables.
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        //flag = hardwareMap.get(DcMotor.class, "flag");
        glypherArmTilt = hardwareMap.get(DcMotor.class, "tilt");
        glypherArmYax = hardwareMap.get(DcMotor.class, "up");
        glypherPinch = hardwareMap.get(DcMotor.class, "pinch");

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // reset encoder here
        glypherPinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glypherPinch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        glypherArmYax.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glypherArmYax.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        glypherArmTilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glypherArmTilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Set Motor Directions
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        glypherArmYax.setDirection(DcMotorSimple.Direction.REVERSE);
        glypherPinch.setDirection(DcMotorSimple.Direction.FORWARD);
        glypherArmTilt.setDirection(DcMotorSimple.Direction.REVERSE);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY

    @Override
    public void init_loop() {
    }
    // Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
    }
    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        gamepadOneStuff();
        gamepadTwoStuff();
        //flag.setPower(1);
        if(runtime.seconds() <= 119){
            stop();
        }
    }

    private void gamepadOneStuff(){
        double leftPower = 0;
        double rightPower = 0;
        double leftBackPower = 0;
        double rightBackPower = 0;

        double leftY = gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;

        double rightY = gamepad1.right_stick_y;
        if(gamepad1.right_bumper || gamepad1.right_trigger > 0) {
            if (rightY == 0) {
                rightPower = leftX;
                leftPower = -leftX;
                leftBackPower = leftX;
                rightBackPower = -leftX;
            }
        } else {
            leftPower = leftBackPower = gamepad1.left_stick_y;
            rightPower = rightBackPower = gamepad1.right_stick_y;
        }

        double powermult = .75;

        if(gamepad1.left_trigger > 0){
            powermult = .25;
        } else if(gamepad1.left_bumper){
            powermult = 1;
        }

        leftDrive.setPower(leftPower*powermult);
        rightDrive.setPower(rightPower*powermult);
        leftBackDrive.setPower(leftBackPower*powermult);
        rightBackDrive.setPower(rightBackPower*powermult);

    }

    private void gamepadTwoStuff(){

        movePinch();
        moveTilt();
        moveYax();

        telemetry.addData("flENC",  leftDrive.getCurrentPosition());
        telemetry.addData("frENC",  rightDrive.getCurrentPosition());
        telemetry.addData("pinchENC",  glypherPinch.getCurrentPosition());
        telemetry.addData("tiltENC",  glypherArmTilt.getCurrentPosition());
        telemetry.addData("yaxENC",  glypherArmYax.getCurrentPosition());
        telemetry.update();
    }

    private void movePinch(){
        double glypherPinchPower = 0;
        final int maxPinch = 0;
        final int minPinch = -350;
        final double pinchSpeed = 1;
        int pinchPosition = glypherPinch.getCurrentPosition();

        if (gamepad2.right_trigger > 0.5) {
            if (pinchPosition > minPinch) {
                glypherPinchPower = -pinchSpeed;
            }
        }
        else if (gamepad2.left_trigger > 0.5) {
            if (pinchPosition < maxPinch) {
                glypherPinchPower = pinchSpeed;
            }
        }

        glypherPinch.setPower(glypherPinchPower);
    }

    private void moveTilt(){

        glypherArmTilt.setPower(gamepad2.right_stick_y * 0.075);

    }

    private void moveYax(){
        double glypherYaxPower = 0;
        final int maxYax = 3987;
        final int minYax = 0;
        final double yaxSpeed = 0.7;
        int yaxPosition = glypherArmYax.getCurrentPosition();

        //if(Math.abs(gamepad2.left_stick_x) < Math.abs(gamepad2.left_stick_y)){
        if(gamepad2.left_stick_y < 0 && yaxPosition < maxYax){
            glypherYaxPower = yaxSpeed;
        }
        else if(gamepad2.left_stick_y > 0 && yaxPosition > minYax){
            glypherYaxPower = -yaxSpeed;
        }
        //}
        glypherArmYax.setPower(glypherYaxPower);

    }


    private void grabRelic(){

    }
    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
        //flag.setPower(0);
    }

}