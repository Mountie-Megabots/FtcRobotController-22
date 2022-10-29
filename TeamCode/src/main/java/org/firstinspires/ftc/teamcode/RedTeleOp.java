package org.firstinspires.ftc.teamcode;

import android.graphics.RenderNode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class RedTeleOp extends LinearOpMode {
    private Blinker control_Hub;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor armMotor;
    private Servo ToucanSam;
    private BNO055IMU imu;
    private DigitalChannel limitSwitch;
    private int armOffset = 0;

    double servoSetting = 0;
    double headingOffset = 0;

    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        ToucanSam = hardwareMap.get(Servo.class, "ToucanSam");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

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

            if( !(limitSwitch.getState()) && gamepad2.left_stick_y > 0){
                resetArmPosition();
                armMotor.setPower(0);
            }
            else{
                armMotor.setPower(gamepad2.left_stick_y);
            }

            ToucanSam.setPosition(servoSetting);
            telemetry.addData("Limit Switch", limitSwitch.getState());
            telemetry.addData("Status", "Running");
            telemetry.addData("Servo-Status",  servoSetting);
            telemetry.addData("Gyro", getHeading());
            telemetry.addData("Arm-Pos", getArmPosition())
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

        frontLeftDrive.setPower(frontLeftPower);
        leftBackDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        rightBackDrive.setPower(backRightPower);
    }

    public double getHeading() {
        Orientation angles =  imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);
        return -angles.firstAngle-headingOffset;
    }

    public void resetArmPosition(){
       armOffset = armMotor.getCurrentPosition();

    }

    public int getArmPosition(){
        int armPosition = 0;

        armPosition = armMotor.getCurrentPosition() - armOffset;
        return armPosition;
    }
}