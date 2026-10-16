package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="TestTeleop", group="Linear OpMode")
public class TestTeleopGoldenEye extends LinearOpMode {
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    DcMotor test = null;

    public void runOpMode() {
        double leftFrontPower = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
        double leftBackPower = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
        double rightFrontPower = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
        double rightBackPower = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
        double speed = 1.0;
        boolean invertDir = false;
        int invDir = 1;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFrontDrive = hardwareMap.get(DcMotor.class,"leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class,"leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class,"rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class,"rightBackDrive");

        test = hardwareMap.get(DcMotor.class, "test");

        waitForStart();
        while (opModeIsActive()) {

            leftFrontDrive.setPower(leftFrontPower * speed * invDir);
            leftBackDrive.setPower(leftBackPower * speed * invDir);
            rightFrontDrive.setPower(rightFrontPower * speed * invDir);
            rightBackDrive.setPower(rightBackPower * speed * invDir);


            test.setPower(gamepad1.right_stick_y * 1.25);

        }
        }
    }



