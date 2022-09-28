/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * TeleOp Mode
 * <p>
 *Enables control of the robot via the gamepad
 */

@TeleOp(name = "Mecanum: Tele Op", group = "Mecanum")
public class MecanumTeleOp extends OpMode {

    private String startDate;
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor w1, w2, w3, w4, fly1, fly2;
    Servo servo1, servo2;

    @Override
    public void init() {
        w1 = hardwareMap.dcMotor.get("mFL");
        w2 = hardwareMap.dcMotor.get("mBL");
        w3 = hardwareMap.dcMotor.get("mFR");
        w4 = hardwareMap.dcMotor.get("mBR");
        fly1 = hardwareMap.dcMotor.get("fly1");
        fly2 = hardwareMap.dcMotor.get("fly2");
        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");



        //w1.setDirection(DcMotor.Direction.REVERSE);
        //w2.setDirection(DcMotor.Direction.REVERSE);
        w3.setDirection(DcMotor.Direction.REVERSE);
        w4.setDirection(DcMotor.Direction.REVERSE);
        fly2.setDirection(DcMotor.Direction.REVERSE);

        servo1.setDirection(Servo.Direction.REVERSE);

        w1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        w2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        w3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        w4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        gamepad1.setJoystickDeadzone(0.2f);
    }

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init_loop() {
        startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
        runtime.reset();
        telemetry.addData("Null Op Init Loop", runtime.toString());
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    int servoState = 0;
    @Override
    public void loop() {

        //add telemetry
        telemetry.addData("1 Start", "NullOp started at " + startDate);
        telemetry.addData("W1 Encoder Value" , w1.getCurrentPosition());
        telemetry.addData("W2 Encoder Value" , w2.getCurrentPosition());
        telemetry.addData("W3 Encoder Value" , w3.getCurrentPosition());
        telemetry.addData("W4 Encoder Value" , w4.getCurrentPosition());


        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double lt = gamepad1.left_trigger;
        double rt = gamepad1.right_trigger;

        if (gamepad1.dpad_left) {
            //move left
            w1.setPower(.3);
            w2.setPower(-.3);
            w3.setPower(-.3);
            w4.setPower(.3);

        } else if (gamepad1.dpad_right) {
            //move right
            w1.setPower(-.3);
            w2.setPower(.3);
            w3.setPower(.3);
            w4.setPower(-.3);

        }else if (gamepad1.dpad_up) {
            //move forward
            w1.setPower(-.3);
            w2.setPower(-.3);
            w3.setPower(-.3);
            w4.setPower(-.3);
        }else if (gamepad1.dpad_down) {
            //move backward
            w1.setPower(.3);
            w2.setPower(.3);
            w3.setPower(.3);
            w4.setPower(.3);

        } else {
            //don't change this
            w1.setPower(y-x+lt-rt);
            w2.setPower(y+x+lt-rt);
            w3.setPower(y+x-lt+rt);
            w4.setPower(y-x-lt+rt);

            if (gamepad1.a) {
                fly1.setPower(1);
                fly2.setPower(1);
            }else {
                fly1.setPower(0);
                fly2.setPower(0);
            }
            if (gamepad1.b){
                if(servoState == 0) {
                    servoState = 1;
                    servo1.setPosition(.8);
                    servo2.setPosition(.8);
                } else {
                    servoState = 0;
                    servo1.setPosition(0);
                    servo2.setPosition(0);
                }



            }
        }
    }
}