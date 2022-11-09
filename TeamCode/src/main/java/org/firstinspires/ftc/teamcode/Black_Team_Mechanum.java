package org.firstinspires.ftc.teamcode;

import android.graphics.RenderNode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
@TeleOp
public class Black_Team_Mechanum extends LinearOpMode {
    private Blinker control_Hub;
    private DcMotor b_left;
    private DcMotor b_right;
    private DcMotor f_left;
    private DcMotor f_right;
    private DcMotor armMotor;
    private Servo servo;
    private BNO055IMU imu;
    double speed = 10;
    double servoSetting = .5;
    private AnalogInput ai1;


    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        b_left = hardwareMap.get(DcMotor.class, "b_left");
        b_right = hardwareMap.get(DcMotor.class, "b_right");
        f_left = hardwareMap.get(DcMotor.class, "f_left");
        f_right = hardwareMap.get(DcMotor.class, "f_right");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        servo = hardwareMap.get(Servo.class, "servo");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        b_left.setDirection(DcMotor.Direction.REVERSE);
        b_right.setDirection(DcMotor.Direction.FORWARD);
        f_left.setDirection(DcMotor.Direction.REVERSE);
        f_right.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            /* Old drive code- Kevin said it was bad
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

            double leftPower = -gamepad1.left_stick_y * (speed / 10);
            double rightPower = -gamepad1.right_stick_y * (speed / 10);

            f_left.setPower(leftPower);
            f_right.setPower(rightPower);
            b_left.setPower(leftPower);
            b_right.setPower(rightPower);

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
            }*/

            /*double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            f_left.setPower(v1);
            f_right.setPower(v2);
            b_left.setPower(v3);
            b_right.setPower(v4);*/

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            f_left.setPower(frontLeftPower);
            b_left.setPower(backLeftPower);
            f_right.setPower(frontRightPower);
            b_right.setPower(backRightPower);


            if (gamepad2.a) {
                armMotor.setPower(0.5);

            } else if (gamepad2.b) {
                armMotor.setPower(-0.5);


            } else {
                armMotor.setPower(0);
            }

            
            if (gamepad2.x) {
                servo.setPosition(1);

            }
            //don't use y and x button (may break servos)
            if (gamepad2.y) {
                servo.setPosition(0);

            }
             /*if (gamepad2.x) {
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


            telemetry.addData("Status", "Running");
            telemetry.addData("Servo-Status",  servoSetting);
            telemetry.addData("Gyro", getHeading());
            telemetry.update();
        }
    }

    public double getHeading() {
        Orientation angles =  imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        return heading;
    }
}