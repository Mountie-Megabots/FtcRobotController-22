package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NWRobot.RobotBase;

@Autonomous
@Disabled
public class AutoBaseDemo extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    RobotBase base;

    @Override
    public void runOpMode() {
        base = new RobotBase(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // Step 1:  Drive forward for 3 seconds
        base.drive(1, 0, 0, false);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Spin right for 1.3 seconds
        base.drive(0, 0, -1, false);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Drive Backward for 1 Second
        base.drive(-1, 0, 0, false);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop
        base.drive(0, 0, 0, false);

        telemetry.addData("Path", "Complete");
        telemetry.update();

        telemetry.addData("Status", "Running");
        telemetry.update();
    }

}