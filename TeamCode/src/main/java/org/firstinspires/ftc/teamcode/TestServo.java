package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class TestServo extends LinearOpMode {
    private Servo servo;
    double servoSetting = .5;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "servo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        double servoPosition = 0;
        while (opModeIsActive()) {

            if (gamepad2.y) {
                servoPosition+= 0.01;
            }
            else if(gamepad2.b){
                servoPosition-= 0.01;
            }

            servo.setPosition(servoPosition);


            telemetry.addData("Status", "Running");
            telemetry.addData("Servo-Status",  servoSetting);
            telemetry.addData("Servo-Pos", servoPosition);
            telemetry.update();
        }
    }


}