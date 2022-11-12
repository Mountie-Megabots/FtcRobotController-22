package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.NWRobot.AprilTagDetector;
import org.firstinspires.ftc.teamcode.NWRobot.RobotBase;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "AprilTagAuto", group = "Concept")
public class AprilTagAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    RobotBase base;
    AprilTagDetector detector;
    int parkingSpace = 0;

    @Override
    public void runOpMode() {
        base = new RobotBase(this);
        detector = new AprilTagDetector(hardwareMap);

        waitForStart();

        // Step 1:  Drive forward for 1 seconds
        base.drive(.25, 0, 0, false);
        runtime.reset();
        while (opModeIsActive() && (base.odometry.currentPosition().getComponents()[0] < 34 || runtime.seconds() < 10)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());

            switch(detector.FindAprilTag()){
                case 100:
                    parkingSpace = 1;
                    break;
                case 200:
                    parkingSpace = 2;
                    break;
                case 300:
                    parkingSpace = 3;
                    break;
            }

            if(parkingSpace == 0){
                telemetry.addData("Parking Space", "Not determined");
            }
            else{
                telemetry.addData("Parking Space", "Targeting space %d", parkingSpace);
            }

            telemetry.update();
        }

        // Step 2:  Stop. Drive to different parking space if necessary.
        base.drive(1, 0, 0, false);
        runtime.reset();
        if(parkingSpace == 1 || parkingSpace == 3) {
            double targetPosition = 0;

            if (parkingSpace == 1){
                targetPosition = -10;
            }

            while (opModeIsActive() && (runtime.seconds() < 1.0)) {
                telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
        }

        // Step 5:  Stop or drive to parking space.
        if( parkingSpace == 1){
            base.drive(0, -1, 0, false);
        }
        else if( parkingSpace == 3){
            base.drive(0, 1, 0, false);
        }
        else {
            base.drive(0, 0, 0, false);
        }
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 5: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

    }

}

