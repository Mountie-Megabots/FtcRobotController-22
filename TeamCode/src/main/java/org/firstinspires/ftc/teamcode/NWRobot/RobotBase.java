package org.firstinspires.ftc.teamcode.NWRobot;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import android.os.Environment;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.Dictionary;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.List;
import java.util.Properties;
import java.util.concurrent.TimeUnit;


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
    public DcMotorEx xEncoder;
    public DcMotorEx yEncoder;
    public Vector2D position; // this has to be inches
    public LinearOpMode opMode;
    public Odometry odometry;
    public Properties prop;
    public Boolean isRedBot = true;
    public int maxArmPosition = -1000;
    public double lastRunTime;
    public Timing.Timer armTimer;

    private int armOffset = 0;
    double servoSetting = 0;
    double headingOffset = 0;

    //Cached sensor readings
    boolean limitSwitchState = false;
    Dictionary<DcMotor,Integer> encoders;
    Orientation IMUOrientation;

    public RobotBase(LinearOpMode opmode){
        this.opMode = opmode;
        control_Hub = opMode.hardwareMap.get(Blinker.class, "Control Hub");
        leftBackDrive = opMode.hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = opMode.hardwareMap.get(DcMotor.class, "rightBackDrive");
        frontLeftDrive = opMode.hardwareMap.get(DcMotor.class, "leftFrontDrive");
        frontRightDrive = opMode.hardwareMap.get(DcMotor.class, "rightFrontDrive");
        armMotor = opMode.hardwareMap.get(DcMotor.class, "armMotor");
        barb = opMode.hardwareMap.get(Servo.class, "servo");
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        limitSwitch = opMode.hardwareMap.get(DigitalChannel.class, "limitSwitch");
        xEncoder = opMode.hardwareMap.get(DcMotorEx.class, "xEncoder");
        yEncoder = opMode.hardwareMap.get(DcMotorEx.class, "yEncoder");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        encoders = new Hashtable<DcMotor,Integer>();
        encoders.put(leftBackDrive, 0);
        encoders.put(rightBackDrive, 0);
        encoders.put(frontLeftDrive, 0);
        encoders.put(frontRightDrive, 0);
        encoders.put(xEncoder, 0);
        encoders.put(yEncoder, 0);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        opmode.telemetry.setAutoClear(false);

        prop = new Properties();

        String filePath = Environment.getExternalStorageDirectory() + "/FIRST/robot.config";
        try (FileInputStream fis = new FileInputStream(filePath)) {
            prop.load(fis);
        } catch (FileNotFoundException ex) {
            opmode.telemetry.addData("Properties:", "File %s not found.", filePath);
        } catch (IOException ex) {
            opmode.telemetry.addData("Properties:", "IO Failed.");
        }

        if(prop.getProperty("RobotName").equals("Redbot")){
            isRedBot = true;
            opmode.telemetry.addData("Properties:", "This is Redbot");
        }
        else{
            isRedBot = false;
            opmode.telemetry.addData("Properties:", "This is Blackbot");
        }

        opmode.telemetry.update();

        if(isRedBot){
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
            frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
            frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        }
        else{
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        }

        List<LynxModule> allHubs = opMode.hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        armTimer = new Timing.Timer(1000, TimeUnit.MILLISECONDS);
        odometry = new Odometry(this);
        odometry.initialize(opMode);

    }

    private void updateSensors(){
        //Update Encoders
        Enumeration<DcMotor> motors = encoders.keys();
        while(motors.hasMoreElements()){
            DcMotor motor = motors.nextElement();
            encoders.put(motor,motor.getCurrentPosition());
        }

        limitSwitchState = limitSwitch.getState();

        IMUOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void enabledPeriodic(){
        //Check limit switch
        this.updateSensors();

        if ( !(limitSwitchState)){
            resetArmPosition();
        }

        odometry.updatePosition();
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

    public void driveWithHeading(double y, double x, double heading){
        double rotX;
        double rotY;
        double rx = 0;
        double headingError;

        double maxRotationalCorrection = .5;
        double minRotationalCorrection = .05;
        double headingDeadbandDegrees = 10;
        double speedPerDegree = (maxRotationalCorrection-minRotationalCorrection)/(180-headingDeadbandDegrees/2);

        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = Math.toRadians(getHeading());

        // if(fieldRelative){
        rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
//
//        }
//        else{
//            rotX = x;
//            rotY = y;
//        }

        headingError = this.angleDifference(getHeading(), heading);
        if(Math.abs(headingError) > headingDeadbandDegrees){
            rx = clamp( speedPerDegree*(headingError-headingDeadbandDegrees/2),minRotationalCorrection, maxRotationalCorrection);
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

    public double angleDifference(double a, double b){
        double d = Math.abs(a - b) % 360;
        double r = d > 180 ? 360 - d : d;

        //calculate sign
        int sign = (a - b >= 0 && a - b <= 180) || (a - b <=-180 && a- b>= -360) ? 1 : -1;
        return r * sign;
    }

    public double getHeading() {
        return -IMUOrientation.firstAngle-headingOffset;
    }

    public void resetHeading(){
        headingOffset = getHeading() + headingOffset;
    }

    public void manuallyMoveArm(double speed){
        if(armMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if ( !(limitSwitchState) && speed > 0){
            armMotor.setPower(0);
        }
        else if(speed < 0 && getCurrentMotorPos(armMotor) <= maxArmPosition){
            armMotor.setPower(0);
        }
        else{
            armMotor.setPower(speed);
        }

    }

    public void moveArmToPosition(int position){
        position = clamp(position, maxArmPosition, 8000);
        armMotor.setTargetPosition(position);

        if(armMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        armMotor.setPower(.5);
    }

    public int stickToPosition(double stick){
        int position = getArmPosition();
        double timeToTopPosition = 5;
        double countsPerSecond = (timeToTopPosition)/maxArmPosition;
        double elapsedTime = opMode.getRuntime() - lastRunTime;
        lastRunTime = opMode.getRuntime();
        /*if(armTimer.isTimerOn()){
            position = (int) (position + armTimer.elapsedTime()*countsPerMillisecond);
        }

        armTimer.*/

        if(elapsedTime < 0.05){
            position = (int) (position + elapsedTime*countsPerSecond);
        }

        return position;
    }

    public void resetArmPosition(){
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armOffset = armMotor.getCurrentPosition();
    }

    public int getArmPosition(){
        return getCurrentMotorPos(armMotor) - armOffset;
    }

    public void barbOff(){
        barb.setPosition(0);
    }

    public void barbOn(){
        barb.setPosition(1);
    }

    public boolean getLimitSwitch(){
        return limitSwitchState;
    }

    public int getCurrentMotorPos(DcMotor motor){
        return motor.getCurrentPosition();
    }
}
