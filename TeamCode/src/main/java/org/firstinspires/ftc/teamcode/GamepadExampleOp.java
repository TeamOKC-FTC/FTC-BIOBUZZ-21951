package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Gamepad Example")
public class GamepadExampleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        //test add for github
        waitForStart();

        while (opModeIsActive()) {
            // --- Button Inputs (true or false) ---
            telemetry.addData("a", gamepad1.a);
            telemetry.addData("b", gamepad1.b);
            telemetry.addData("x", gamepad1.x);
            telemetry.addData("y", gamepad1.y);
            telemetry.addData("left_bumper", gamepad1.left_bumper);
            telemetry.addData("right_bumper", gamepad1.right_bumper);
            telemetry.addData("dpad_up", gamepad1.dpad_up);
            telemetry.addData("dpad_down", gamepad1.dpad_down);
            telemetry.addData("dpad_left", gamepad1.dpad_left);
            telemetry.addData("dpad_right", gamepad1.dpad_right);

            // --- Axis Inputs (a number from -1.0 to 1.0, or 0.0 to 1.0 for triggers) ---
            telemetry.addData("left_stick_x", gamepad1.left_stick_x);
            telemetry.addData("left_stick_y", gamepad1.left_stick_y);
            telemetry.addData("right_stick_x", gamepad1.right_stick_x);
            telemetry.addData("right_stick_y", gamepad1.right_stick_y);
            telemetry.addData("left_trigger", gamepad1.left_trigger);
            telemetry.addData("right_trigger", gamepad1.right_trigger);

            telemetry.update();
        }
    }
}