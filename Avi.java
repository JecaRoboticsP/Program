package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "Avi",group = "Linear OpMode")
public class Avi extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
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

        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            int speed = 1;
            double y_input = -gamepad1.left_stick_y;
            double x_input = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            telemetry.addLine(gamepad1.left_stick_y + " " + gamepad1.left_stick_x);
            telemetry.update();
            motorFR.setPower((y_input * speed) - (x_input * speed) - (turn * speed));
            motorBR.setPower((y_input * speed) + (x_input * speed) - (turn * speed));
            motorFL.setPower((y_input * speed) + (x_input * speed) + (turn * speed));
            motorBL.setPower((y_input * speed) - (x_input * speed) + (turn * speed));

            /*
            motorFR.setPower((gamepad1.left_stick_y * speed) - (gamepad1.left_stick_x * speed) + gamepad1.right_trigger - gamepad1.left_trigger + (gamepad1.right_stick_x * speed));
            motorBR.setPower((gamepad1.left_stick_y * speed) + (gamepad1.left_stick_x * speed) - gamepad1.right_trigger + gamepad1.left_trigger + (gamepad1.right_stick_x * speed));
            motorFL.setPower(-(gamepad1.left_stick_y * speed) + (gamepad1.left_stick_x * speed) + gamepad1.right_trigger + gamepad1.left_trigger + (gamepad1.right_stick_x * speed));
            motorBL.setPower(-(gamepad1.left_stick_y * speed) - (gamepad1.left_stick_x * speed) - gamepad1.right_trigger - gamepad1.left_trigger + (gamepad1.right_stick_x * speed));
            */
        }
    }
}