package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "XDriveRed")
public class HexDriveRed extends LinearOpMode{
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        waitForStart();

        while (opModeIsActive()) {
            simpleDrive();
        }
    }
    public void simpleDrive(){
        double power = .5;
        if (gamepad1.dpad_up) {
            leftFrontDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(-power);
            rightFrontDrive.setPower(-power);
        } else if (gamepad1.dpad_left) {
            leftFrontDrive.setPower(power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(power);
            rightFrontDrive.setPower(-power);
        } else if (gamepad1.dpad_down) {
            leftFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(power);
            rightFrontDrive.setPower(power);
        } else if (gamepad1.dpad_right) {
            leftFrontDrive.setPower(-power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(-power);
            rightFrontDrive.setPower(power);
        } else if (Math.abs(gamepad1.right_stick_x) > 0) {
            leftFrontDrive.setPower(-gamepad1.right_stick_x);
            leftBackDrive.setPower(-gamepad1.right_stick_x);
            rightBackDrive.setPower(gamepad1.right_stick_x);
            rightFrontDrive.setPower(gamepad1.right_stick_x);
        } else {
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
        }
    }



    public void drive() {
        double Protate = gamepad1.right_stick_x/4;
        double stick_x = gamepad1.left_stick_x * Math.sqrt(Math.pow(1-Math.abs(Protate) , 2)/2);
        double stick_y = gamepad1.left_stick_y * Math.sqrt(Math.pow(1-Math.abs(Protate) , 2)/2);
        double theta = 0;
        double Px = 0;
        double Py = 0;

        theta = Math.atan2(stick_y, stick_x) - (Math.PI / 2);
        Px = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta + Math.PI /
                4));
        Py = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta - Math.PI /
                4));

        telemetry.addData("Stick_X", stick_x);
        telemetry.addData("Stick_Y", stick_y);
        telemetry.addData("Magnitude", Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)));
        telemetry.addData("Front Left", Py - Protate);
        telemetry.addData("Back Left", Px - Protate);
        telemetry.addData("Back Right", Py + Protate);
        telemetry.addData("Front Right", Px + Protate);
        leftFrontDrive.setPower(Py - Protate);
        leftBackDrive.setPower(Px - Protate);
        rightFrontDrive.setPower(Py + Protate);
        rightBackDrive.setPower(Px + Protate);
    }

}
