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

@Autonomous(name = "AprilTagAuto")
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
            telemetry.setAutoClear(true);

            //Lift arm
            if(autoStage == 0){
                base.moveArmToPosition(-1200);
                if(base.getArmPosition() < -1000){
                    runtime.reset();
                    autoStage++;
                }
            }
            //Start detecting AprilTags
            if(autoStage == 1){
                if(runtime.seconds() < 2){
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
                }
                else{
                    runtime.reset();
                    autoStage++;
                }

            }
            //Drive Forward
            if(autoStage == 2){
                if( !base.driveToPosition(50,0,0,.5,.2)) {
                    telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                    telemetry.addData("Heading", base.getHeading());
                    telemetry.addData("X", base.odometry.currentPosition().getComponents()[0]);
                    telemetry.addData("Y", base.odometry.currentPosition().getComponents()[1]);

                    if(parkingSpace == 0) {
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
                        telemetry.addData("Parking Space", "Not determined");
                    }
                    else {
                        telemetry.addData("Parking Space", "Targeting space %d", parkingSpace);
                    }

                }
                else{
                    base.drive(0,0,0, false);
                    runtime.reset();
                    autoStage++;
                    /*if(parkingSpace == 1){
                        autoStage=3;
                    }
                    else if(parkingSpace == 3){
                        autoStage=4;
                    }
                    else{
                        autoStage=5;
                    }*/
                }
            }
            //Turn To High Pole
            if(autoStage == 3){
                if( !base.driveToPosition(50,0,46,.5,.2)) {
                    telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                    telemetry.addData("Heading", base.getHeading());
                    telemetry.addData("X", base.odometry.currentPosition().getComponents()[0]);
                    telemetry.addData("Y", base.odometry.currentPosition().getComponents()[1]);


                }
                else{
                    base.drive(0,0,0, false);
                    runtime.reset();

                        autoStage=4;
                }
                //Raise Arm
                if(autoStage == 4) {
                    base.moveArmToPosition(-5868);
                    if (base.getArmPosition() < -5800) {
                        runtime.reset();
                        base.barbOff();
                        autoStage++;
                    }
                }
            }
            //Parking Space 1
            /*else if(autoStage == 3){
                if( !base.driveToPosition(50, -23, 0,.4,.2)) {
                    telemetry.addData("Path", "Driving to Parking Space 1: %4.1f S Elapsed", runtime.seconds());
                    telemetry.addData("Heading", base.getHeading());
                    telemetry.addData("X", base.odometry.currentPosition().getComponents()[0]);
                    telemetry.addData("Y", base.odometry.currentPosition().getComponents()[1]);
                }
                else{
                    base.drive(0,0,0, false);
                    runtime.reset();
                    autoStage=5;
                }
            }
            //Parking Space 3
            else if(autoStage == 4){
                if( !base.driveToPosition(50, 23, 0,.4,.2)) {
                    telemetry.addData("Path", "Driving to Parking Space 3: %4.1f S Elapsed", runtime.seconds());
                    telemetry.addData("Heading", base.getHeading());
                    telemetry.addData("X", base.odometry.currentPosition().getComponents()[0]);
                    telemetry.addData("Y", base.odometry.currentPosition().getComponents()[1]);
                }
                else{
                    base.drive(0,0,0, false);
                    runtime.reset();
                    autoStage=5;
                }
            }*/
            //Done
            else if(autoStage == 5){
                    telemetry.addData("Path", "Reached Parking Space %d!",parkingSpace);

                    base.drive(0,0,0,false);
                    runtime.reset();
            }


            telemetry.update();
        }

    }

}

