package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.NWRobot.RobotBase;

@TeleOp(name = "TeleOp Base")
public class TelepBaseDemo extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotBase base = new RobotBase(this);

        //telemetry.addData("Status", "Initialized");
        //telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x; // Counteracts imperfect strafing
            double rx = gamepad1.right_stick_x;
            boolean rightBumper = gamepad1.right_bumper;

            base.enabledPeriodic();

            double Xodommetry = base.odometry.currentPosition().getComponents()[0];
            double Yodommetry = base.odometry.currentPosition().getComponents()[1];

            if(rightBumper){
                base.drive(y, x, rx/4, false);
            }
            else{
                base.drive(y, x, rx*.75, false);
            }


            if(gamepad1.a){
                base.resetHeading();
            }

            if (gamepad2.x) {
                base.barbOff();
            }
            else if (gamepad2.y) {
                base.barbOn();
            }

            if (gamepad2.a) {
                base.moveArmToPosition(-4000);
            }
            else if (gamepad2.b) {
                base.moveArmToPosition(-8000);
            }
            else if (Math.abs(gamepad2.left_stick_y) > 0.05){
                base.manuallyMoveArm(gamepad2.left_stick_y);
            }
            else{
                base.moveArmToPosition(base.stickToPosition(gamepad2.right_stick_y));
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("Limit Switch", base.getLimitSwitch());
            telemetry.addData("Gyro", base.getHeading());
            telemetry.addData("Arm-Pos", base.getArmPosition());
            telemetry.addData("Field pos-X:",Xodommetry);
            telemetry.addData("Field pos-Y:",Yodommetry);
            telemetry.update();
        }
    }

}