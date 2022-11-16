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
    int autoStage = 0;

    @Override
    public void runOpMode() {
        base = new RobotBase(this);
        detector = new AprilTagDetector(hardwareMap);

        waitForStart();

        // Step 1:  Drive forward for 1 seconds

        runtime.reset();
        base.resetHeading();
        while (opModeIsActive()) {
            base.enabledPeriodic();

            if(autoStage == 0){
                base.driveWithHeading(.25, 0, 0);
                if( base.odometry.currentPosition().getComponents()[0] < 34) {
                    telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                    telemetry.addData("Heading", base.getHeading());
                    telemetry.addData("X", base.odometry.currentPosition().getComponents()[0]);
                    telemetry.addData("Y", base.odometry.currentPosition().getComponents()[1]);

                    switch (detector.FindAprilTag()) {
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

                    if (parkingSpace == 0) {
                        telemetry.addData("Parking Space", "Not determined");
                    } else {
                        telemetry.addData("Parking Space", "Targeting space %d", parkingSpace);
                    }

                    telemetry.update();
                }
                else{
                    base.driveWithHeading(0,0,0);
                    runtime.reset();
                    if(parkingSpace == 1){
                        autoStage=1;
                    }
                    else if(parkingSpace == 3){
                        autoStage=2;
                    }
                    else{
                        autoStage=3;
                    }
                }
            }
            //Parking Space 1
            else if(autoStage == 1){
                base.driveWithHeading(0, -.25, 0);
                if( base.odometry.currentPosition().getComponents()[1] > - 12) {
                    telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                    telemetry.addData("Heading", base.getHeading());
                    telemetry.addData("X", base.odometry.currentPosition().getComponents()[0]);
                    telemetry.addData("Y", base.odometry.currentPosition().getComponents()[1]);
                }
                else{
                    base.driveWithHeading(0,0,0);
                    runtime.reset();
                    autoStage=3;
                }
            }
            //Parking Space 3
            else if(autoStage == 2){
                base.driveWithHeading(0, .25, 0);
                if( base.odometry.currentPosition().getComponents()[1] > 12) {
                    telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.addData("Heading", base.getHeading());
                    telemetry.addData("X", base.odometry.currentPosition().getComponents()[0]);
                    telemetry.addData("Y", base.odometry.currentPosition().getComponents()[1]);
                }
                else{
                    base.driveWithHeading(0,0,0);
                    runtime.reset();
                    autoStage=3;
                }
            }
            //Done
            else if(autoStage == 3){
                base.driveWithHeading(0, .25, 0);
                if( base.odometry.currentPosition().getComponents()[1] > 12) {
                    telemetry.addData("Path", "Reached Parking Space %d!",parkingSpace);
                }
                else{
                    base.drive(0,0,0,false);
                    runtime.reset();

                }
            }


            telemetry.update();
        }

    }

}

