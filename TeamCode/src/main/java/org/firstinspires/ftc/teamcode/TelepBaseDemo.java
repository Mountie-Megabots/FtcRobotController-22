package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.NWRobot.RobotBase;

@TeleOp(name = "TeleOp")
public class TelepBaseDemo extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotBase base = new RobotBase(this);

        //telemetry.addData("Status", "Initialized");
        //telemetry.update();
        base.armSetPosition = base.getArmPosition();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.setAutoClear(true);

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x; // Counteracts imperfect strafing
            double rx = gamepad1.right_stick_x;
            boolean rightBumper = gamepad1.right_bumper;

            base.enabledPeriodic();

            double Xodommetry = base.odometry.currentPosition().getComponents()[0];
            double Yodommetry = base.odometry.currentPosition().getComponents()[1];

            if(rightBumper){
                base.drive(y/4, x/4, rx/4, true);
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
                base.moveArmToPosition(-5868);
            }
            else if (gamepad2.b && !gamepad2.start) {
                base.moveArmToPosition(-3735);
            }
            else if (Math.abs(gamepad2.left_stick_y) > 0.05){
                base.manuallyMoveArm(gamepad2.left_stick_y);
            }
            else if (Math.abs(gamepad2.right_stick_y) > 0.05){
                base.manuallyMoveArm(gamepad2.right_stick_y/3);
            }
            else if(base.armMoving){
                base.armSetPosition = base.getArmPosition();
                base.armMoving = false;
                base.moveArmToPosition(base.armSetPosition);
            }
            else{
                //base.moveArmToPosition(base.stickToPosition(gamepad2.right_stick_y));
                base.moveArmToPosition(base.armSetPosition);
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