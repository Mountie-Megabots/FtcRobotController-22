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
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;

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
@TeleOp

public class SecodComp1111 extends LinearOpMode {
    private Blinker control_Hub;
    private Blinker expansion_Hub_1;
    private DcMotor backleft;
    private DcMotor backright;
    private DcMotor clawmotor;
    private DcMotor duckmotor;
    private DcMotor intakemotor;
    private DcMotor frontleft;
    private DcMotor frontright;
    private Gyroscope imu;
    private int armflag;
    private TouchSensor armstop;


    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_1 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        clawmotor = hardwareMap.get(DcMotor.class, "clawmotor");
        duckmotor = hardwareMap.get(DcMotor.class, "duckmotor");
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        intakemotor = hardwareMap.get(DcMotor.class, "intakemotor");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        armstop = hardwareMap.get(TouchSensor.class, "armstop");
        
        armflag = 0;
        
        clawmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawmotor.setTargetPosition(0);
        clawmotor.setPower(0.5);
        clawmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            frontleft.setPower(gamepad1.left_stick_y*-1);
            frontright.setPower(gamepad1.right_stick_y*1);
            backleft.setPower(gamepad1.left_stick_y*-1);
            backright.setPower(gamepad1.right_stick_y*1);
            //manual arm
            if (armstop.isPressed() || clawmotor.getCurrentPosition() < -3000){
                if (-gamepad2.right_stick_y < 0.5){
                    clawmotor.setPower(0);
                }
                else{
                    clawmotor.setPower(-gamepad2.right_stick_y*1);
                }
            }
            else if (clawmotor.getCurrentPosition() > -500){
                if (-gamepad2.right_stick_y > -0.5){
                    clawmotor.setPower(0);
                }
                else{
                    clawmotor.setPower(-gamepad2.right_stick_y*1);
                }
            }
            else{
                clawmotor.setPower(-gamepad2.right_stick_y*1);
            }
            
            
            /*if(!armstop.isPressed()){
                clawmotor.setPower(-gamepad2.right_stick_y*1);
            }
            else{
                if (gamepad2.right_stick_y < 0.5){
                    clawmotor.setPower(0);
                }
                else{
                    clawmotor.setPower(-gamepad2.right_stick_y*1);
                }
            }*/

            if (gamepad2.dpad_up){
                armflag = 1;
                telemetry.addLine("No encoders");
                clawmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (gamepad2.dpad_down){
                armflag = 0;
                telemetry.addLine("Encoders");
                clawmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            //arm heights
            if(gamepad2.a && armflag ==0){
                //clawmotor.setTargetPosition(0);
                //clawmotor.setPower(1);
                clawmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.addData("Position", clawmotor.getCurrentPosition());
            }
            if(gamepad2.b && armflag ==0){
                //clawmotor.setTargetPosition(-225);
                //clawmotor.setPower(1);
                clawmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad2.y && armflag ==0){
                //clawmotor.setTargetPosition(-70);
                //clawmotor.setPower(1);
                clawmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            //if(gamepad2.x){
             //   clawmotor.setTargetPosition(570);
             //   clawmotor.setPower(1);
             //   clawmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //}
            
            
            
            //spins carousel
            duckgo(0.5);
            //duckmotor.setPower(gamepad2.left_trigger*0.5);
            //duckmotor.setPower(-gamepad2.right_trigger*0.5);
            
            if(gamepad2.left_bumper){
                intakemotor.setPower(1);
            }
            else if(gamepad2.right_bumper){
                intakemotor.setPower(-1);
            }
            else{
                intakemotor.setPower(0);
            }
            telemetry.addData("Status", "Running");
            telemetry.addData("Stick", -gamepad2.right_stick_y*0.5);
            telemetry.addData("ClawPosition", clawmotor.getCurrentPosition());
            telemetry.addData("Pressed", armstop.isPressed());
            telemetry.update();

        }
    }
    private void duckgo(double duckpwr){
        if (gamepad2.left_trigger > 0.5){
            duckmotor.setPower(duckpwr);
        }
        else if (gamepad2.right_trigger > 0.5){
            duckmotor.setPower(-duckpwr);
        }
        else{
            duckmotor.setPower(0);
        }
        
    }
}
