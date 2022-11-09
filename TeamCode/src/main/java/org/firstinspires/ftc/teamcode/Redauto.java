package org.firstinspires.ftc.teamcode;

import android.graphics.RenderNode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.NWRobot.RobotBase;

@Autonomous(name = "AutoLeft")
public class Redauto extends LinearOpMode {
    private RobotBase bot;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        bot = new RobotBase(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // Step 1:  Drive left for 3 seconds
        bot.drive (0, -1, 0, false);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.25)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop
        bot.drive(0, 0, 0, false);

        telemetry.addData("Path", "Complete");
        telemetry.update();

        telemetry.addData("Status", "Running");
        telemetry.addData("Gyro", bot.getHeading());
        telemetry.update();
    }

}