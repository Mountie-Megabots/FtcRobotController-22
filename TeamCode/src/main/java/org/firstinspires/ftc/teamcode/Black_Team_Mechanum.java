package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Black_Team_Mechanum extends LinearOpMode {
    private Blinker control_Hub;
    private DcMotor b_left;
    private DcMotor b_right;
    private DcMotor f_left;
    private DcMotor f_right;
    private DcMotor armMotor;
    private Servo servoRight;
    private Servo servoLeft;
    private Gyroscope imu;
    double speed = 10;
    double servoSetting = .5;

    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        b_left = hardwareMap.get(DcMotor.class, "b_left");
        b_right = hardwareMap.get(DcMotor.class, "b_right");
        f_left = hardwareMap.get(DcMotor.class, "f_left");
        f_right = hardwareMap.get(DcMotor.class, "f_right");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        servoRight = hardwareMap.get(Servo.class, "servoRight");
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        imu = hardwareMap.get(Gyroscope.class, "imu");

        b_left.setDirection(DcMotor.Direction.REVERSE);
        b_right.setDirection(DcMotor.Direction.FORWARD);
        f_left.setDirection(DcMotor.Direction.REVERSE);
        f_right.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0.1) {
                double rightSlide = (gamepad1.right_trigger * 0.7) + (speed / 20);
                f_left.setPower(rightSlide);
                f_right.setPower(-rightSlide);
                b_left.setPower(-rightSlide);
                b_right.setPower(rightSlide);
            }
            if (gamepad1.left_trigger > 0.1) {
                double leftSlide = (gamepad1.left_trigger * 0.7) + (speed / 20);
                f_left.setPower(-leftSlide);
                f_right.setPower(leftSlide);
                b_left.setPower(leftSlide);
                b_right.setPower(-leftSlide);
            }
            if (gamepad1.dpad_up) {
                if (speed < 10) {
                    speed = speed + 1;
                    sleep(100);
                }
            }
            if (gamepad1.dpad_down) {
                if (speed > 0) {
                    speed = speed - 1;

                }
            }
            if (gamepad2.a) {
                armMotor.setPower(0.5);

            } else if (gamepad2.b) {
                armMotor.setPower(-0.5);


            } else {
                armMotor.setPower(0);
            }


             if (gamepad2.x) {
                servoRight.setPosition(0.25);
                servoLeft.setPosition(0.5);
            }
            //don't use y and x button (may break servos)
            if (gamepad2.y) {
                servoRight.setPosition(0);
                servoLeft.setPosition(0.75);
            }


            /*
            if( gamepad2.x){
                servoSetting = servoSetting + .01;
            }
            else if( gamepad2.y){
                servoSetting-= .01;
            }

            servoLeft.setPosition(servoSetting);
            servoRight.setPosition(servoSetting);*/


            double leftPower = -gamepad1.left_stick_y * (speed / 10);
            double rightPower = -gamepad1.right_stick_y * (speed / 10);

            f_left.setPower(leftPower);
            f_right.setPower(rightPower);
            b_left.setPower(leftPower);
            b_right.setPower(rightPower);

            telemetry.addData("Status", "Running");
            telemetry.addData("Servo-Status",  servoSetting);
            telemetry.update();
        }
    }
}