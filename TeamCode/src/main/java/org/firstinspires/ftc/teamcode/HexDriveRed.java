package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.camera.names.BuiltinCameraNameImpl;

@Disabled
@TeleOp(name = "XDriveRed")
public class HexDriveRed extends LinearOpMode{
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armMotor = null;
    /*private Servo Toucan = null;
    private Servo Sam = null;*/
    private CRServo ToucanSam = null;
    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        //Toucan = hardwareMap.get(Servo.class, "Toucan");
        //Sam = hardwareMap.get(Servo.class, "Sam");
        ToucanSam = hardwareMap.get(CRServo.class, "ToucanSam");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
        waitForStart();

        while (!opModeIsActive()) {
        }

        while (opModeIsActive()) {

            //Move arm
            armMotor.setPower(gamepad2.left_stick_y);

            double heading = getHeading();
            telemetry.addData("imu",heading);
            
            /* servo tuning code
            Toucan.setPosition(gamepad2.right_stick_y);
            Sam.setPosition(gamepad2.right_stick_y);
            */

            // Close Claw
            if(gamepad2.right_bumper){
                //Toucan.setPosition(1.0);
                //Sam.setPosition(.1);
                ToucanSam.setPower(1.0);
            }
            //Open Claw
            else if(gamepad2.left_bumper){
                //Toucan.setPosition(0);
                //Sam.setPosition(1);
                ToucanSam.setPower(-1.0);
            }
            else{
                ToucanSam.setPower(0);
            }
            telemetry.addData("Joystick-Y",gamepad2.right_stick_y);
            telemetry.addData("Joystick-Y",gamepad2.right_stick_y);


            if (gamepad1.right_bumper) {
                driveSimple();
            }
            else {
                drive();
            }

            telemetry.update();
        }
    }

    public void driveSimple(){
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



    public void trigDrive() {
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

    public void drive(){
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  -gamepad1.left_stick_x;
        double yaw     =  -gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // This is test code:
        //
        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public double getHeading(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        return heading;
    }

}
    