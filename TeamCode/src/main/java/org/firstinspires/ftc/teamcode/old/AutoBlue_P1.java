/*
Copyright 2021 FIRST Tech Challenge Team 8142

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@Autonomous
@Disabled
public class AutoBlue_P1 extends LinearOpMode {
    private Blinker control_Hub;
    private Blinker expansion_Hub_1;
    private DcMotorEx backleft;
    private DcMotorEx backright;
    private DcMotor clawmotor;
    private DcMotor duckmotor;
    private DcMotorEx frontleft;
    private DcMotorEx frontright;
    private BNO055IMU imu;
    private DcMotor intakemotor;


    @Override
    public void runOpMode() {
        //gyro initialization
        BNO055IMU.Parameters imuParameters;
        Orientation angles;
        Acceleration gravity;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);
        
        
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_1 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        backleft = hardwareMap.get(DcMotorEx.class, "backleft");
        backright = hardwareMap.get(DcMotorEx.class, "backright");
        clawmotor = hardwareMap.get(DcMotor.class, "clawmotor");
        duckmotor = hardwareMap.get(DcMotor.class, "duckmotor");
        frontleft = hardwareMap.get(DcMotorEx.class, "frontleft");
        frontright = hardwareMap.get(DcMotorEx.class, "frontright");
        intakemotor = hardwareMap.get(DcMotor.class, "intakemotor");
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        


        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            //forward two squares
            raisearm(0);
            forward(1850, 0.35);
            sleep(4500);
            stopmotors();
            
            //turn right
            turnright(-45,0.2);
            sleep(2000);
            stopmotors();
            
            //move towards goal
            forward(500,0.25);
            //arm deposit
            raisearm(-1350);
            sleep(1500);
            blockout(1);
            sleep(2000);
            blockout(0);
            raisearm(0);
            
            //backwards into warehouse
            forward(-400,-0.25);
            sleep(2000);
            stopmotors();
            forward(-3500,1);
            sleep(2500);
            stopmotors();
            /*forward(-650,0.25);
            sleep(3000);
            stopmotors();
            duckmotor.setPower(-0.4);
            sleep(4000);
            duckmotor.setPower(0);
            
            //forward(500,0.5);
            //sleep(1000);
            forward(50,0.2);
            sleep(1000);
            stopmotors();
            turnleft(50,-0.25);
            sleep(2000);
            forward(1000,0.25);
            sleep(7000);
            stopmotors();*/
            sleep(30000);
            
            
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
        
    }
    
    private void duckGo(double pwr){
        duckmotor.setPower(pwr);
    }
    
    private void raisearm(int pos){
        clawmotor.setTargetPosition(pos);
        clawmotor.setPower(0.5);
        clawmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void blockout(double ipwr){
        intakemotor.setPower(ipwr);
    }
    
    private void forward(int squares, double pwr){
        frontleft.setTargetPosition(squares);
        frontleft.setPower(pwr);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

             // front right motor
        frontright.setTargetPosition(-squares);
        frontright.setPower(pwr);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // back left motor
        backleft.setTargetPosition(squares);
        backleft.setPower(pwr);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             // back right motor
        backright.setTargetPosition(-squares);
        backright.setPower(pwr);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    private void turnright(int to_angle, double gpwr){
        BNO055IMU.Parameters imuParameters;
        Orientation angles;
        Acceleration gravity;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
            AxesOrder.ZYX, AngleUnit.DEGREES);
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (angles.firstAngle > to_angle){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
            AxesOrder.ZYX, AngleUnit.DEGREES);
            frontleft.setPower(gpwr);
            backleft.setPower(gpwr);
            frontright.setPower(gpwr);
            backright.setPower(gpwr);
            telemetry.addLine("Turning...");
            telemetry.addData("rot about Z", angles.firstAngle);
            telemetry.update();
        }
        stopmotors();
        
    }
    
    private void stopmotors(){
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
    }
    
    private void turnleft(int to_angle, double gpwr){
        BNO055IMU.Parameters imuParameters;
        Orientation angles;
        Acceleration gravity;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
            AxesOrder.ZYX, AngleUnit.DEGREES);
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (angles.firstAngle < to_angle){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
            AxesOrder.ZYX, AngleUnit.DEGREES);
            frontleft.setPower(gpwr);
            backleft.setPower(gpwr);
            frontright.setPower(gpwr);
            backright.setPower(gpwr);
            telemetry.addLine("Turning...");
            telemetry.addData("rot about Z", angles.firstAngle);
            telemetry.update();
        }
        stopmotors();
        
    }
}
