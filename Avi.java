package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "Avi",group = "Linear OpMode")
public class Avi extends LinearOpMode {

    public DcMotor motorFR;
    public DcMotor motorFL;
    public DcMotor motorBR;
    public DcMotor motorBL;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");

        while (opModeIsActive()) {

            int speed = 1;

            motorFR.setPower((gamepad1.left_stick_y * speed) + (gamepad1.left_stick_x * speed) + gamepad1.right_trigger - gamepad1.left_trigger - (gamepad1.right_stick_x * speed));
            motorBR.setPower((gamepad1.left_stick_y * speed) - (gamepad1.left_stick_x * speed) - gamepad1.right_trigger + gamepad1.left_trigger - (gamepad1.right_stick_x * speed));
            motorFL.setPower((gamepad1.left_stick_y * speed) - (gamepad1.left_stick_x * speed) - gamepad1.right_trigger + gamepad1.left_trigger + (gamepad1.right_stick_x * speed));
            motorBL.setPower((gamepad1.left_stick_y * speed) + (gamepad1.left_stick_x * speed) + gamepad1.right_trigger - gamepad1.left_trigger + (gamepad1.right_stick_x * speed));

        }
    }
}
