package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.NWRobot.RobotBase;

@TeleOp
public class TelepBaseDemo extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotBase base = new RobotBase(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x; // Counteracts imperfect strafing
            double rx = gamepad1.right_stick_x;

            base.enabledPeriodic();

            base.drive(y, x, rx, false);

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
            else{
                base.manuallyMoveArm(gamepad2.left_stick_y);
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("Limit Switch", base.getLimitSwitch());
            telemetry.addData("Gyro", base.getHeading());
            telemetry.addData("Arm-Pos", base.getArmPosition());
            telemetry.update();
        }
    }

}