package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NWRobot.AprilTagDetector;
import org.firstinspires.ftc.teamcode.NWRobot.RobotBase;

@Autonomous
@Disabled
public class DriveToPositionTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    RobotBase base;

    @Override
    public void runOpMode() {
        base = new RobotBase(this);
        int autoStage = 0;
        waitForStart();

        // Step 1:  Drive forward for 1 seconds

        runtime.reset();
        base.resetHeading();
        while (opModeIsActive()) {
            base.enabledPeriodic();
            telemetry.setAutoClear(true);

            if(autoStage == 0){
                if(base.driveToPosition(0,50,0,.5,.15)){
                    autoStage++;
                }
            }
            //Parking Space 1
            else if(autoStage == 1){
                if(base.driveToPosition(0,25,0,.5,.15)){
                    autoStage++;
                }
            }
            //Parking Space 3
            else if(autoStage == 2){
                if(base.driveToPosition(0,25,90,.5,.15)){
                    autoStage++;
                }
            }
            else if(autoStage == 3){
                if(base.driveToPosition(0,0,90,.5,.15)){
                    autoStage++;
                }
            }
            //Done
            else if(autoStage == 4){
                telemetry.addData("Path", "Did It!");

                base.drive(0,0,0,false);
                runtime.reset();
            }


            telemetry.update();
        }

    }

}