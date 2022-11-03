package org.firstinspires.ftc.teamcode.NWRobot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class RobotBase {
    private Blinker control_Hub;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor armMotor;
    private Servo barb;
    private BNO055IMU imu;
    private DigitalChannel limitSwitch;

    private int armOffset = 0;

    double servoSetting = 0;
    double headingOffset = 0;

    public RobotBase(HardwareMap hardwareMap){
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        barb = hardwareMap.get(Servo.class, "ToucanSam");
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
    }

    public void enabledPeriodic(){
        //Check limit switch
        if ( !(limitSwitch.getState())){
            resetArmPosition();
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

    public void resetHeading(){
        headingOffset = getHeading() + headingOffset;
    }

    public void manuallyMoveArm(double speed){
        if(armMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if ( !(limitSwitch.getState())&& speed > 0){
            armMotor.setPower(0);
        }
        else{
            armMotor.setPower(speed);
        }

    }

    public void moveArmToPosition(int position){
        if(armMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        armMotor.setPower(.5);
        armMotor.setTargetPosition(position);
    }

    public void resetArmPosition(){
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armOffset = armMotor.getCurrentPosition();
    }

    public int getArmPosition(){
        int armPosition = 0;

        armPosition = armMotor.getCurrentPosition() - armOffset;
        return armPosition;
    }

    public void barbOff(){
        barb.setPosition(0);
    }

    public void barbOn(){
        barb.setPosition(1);
    }

    public boolean getLimitSwitch(){
        return limitSwitch.getState();
    }
}