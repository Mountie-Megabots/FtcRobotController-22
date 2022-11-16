package org.firstinspires.ftc.teamcode;

import android.graphics.RenderNode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
public class BlackTeleOp extends LinearOpMode {
    private Blinker control_Hub;
    private DcMotor b_left;
    private DcMotor b_right;
    private DcMotor f_left;
    private DcMotor f_right;
    private DcMotor armMotor;
    private Servo servo;
    private BNO055IMU imu;
    double servoSetting = 0;
    double headingOffset = 0;

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

        b_left.setDirection(DcMotor.Direction.FORWARD);
        b_right.setDirection(DcMotor.Direction.REVERSE);
        f_left.setDirection(DcMotor.Direction.FORWARD);
        f_right.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            drive(y, x, rx, false);

            if(gamepad1.a){
                headingOffset = getHeading() + headingOffset;
            }

            if (gamepad2.x) {
                servoSetting = 1;
            }
            else if (gamepad2.y) {
                servoSetting = 0;
            }

            armMotor.setPower(gamepad2.left_stick_y);

            servo.setPosition(servoSetting);
            telemetry.addData("Status", "Running");
            telemetry.addData("Servo-Status",  servoSetting);
            telemetry.addData("Gyro", getHeading());
            telemetry.update();
        }
    }

    public void drive(double y, double x, double rx, boolean fieldRelative){
        double rotX;
        double rotY;

        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = Math.toRadians(getHeading());

        if(fieldRelative){
            rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        }
        else{
            rotX = x;
            rotY = y;
        }


        telemetry.addData("RotX", rotX);
        telemetry.addData("RotY", rotY);

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
    }

    public double getHeading() {
        Orientation angles =  imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);
        return -angles.firstAngle-headingOffset;
    }
}