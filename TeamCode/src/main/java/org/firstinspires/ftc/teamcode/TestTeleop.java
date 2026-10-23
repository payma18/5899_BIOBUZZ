package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="TestTeleop", group="Linear OpMode")
public class TestTeleop extends LinearOpMode {
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
     private DcMotor intake;
    // private DcMotor outtake;


    @Override
    public void runOpMode() {
        double speed = 1.0;      // used to implement half speed; set to full on default
        boolean invertDir = false; // used to invert drive prompts
        int invDir = 1;  // used to activate inverted direction
        boolean keyA = false, keyB = false; // used for toggle keys


        // Initialize the hardware variables; Note that the name in quotes corresponds
        // to the name assigned in the robot configuration
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        // outtake = hardwareMap.get(DcMotor.class,"outtake");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "Left stick: Move");
        telemetry.addData("Controls", "Right stick X: Rotate");
        telemetry.addData("Controls", "Left bumper: Half speed");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            double leftFrontPower = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            double leftBackPower = gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
            double rightFrontPower = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
            double rightBackPower = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            double max;

            // Normalize the values so no wheel power exceeds 100%.
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            if (gamepad1.a) {
                if (keyA == false) {
                    keyA = !keyA;
                    switch ((int) (speed * 10)) {
                        case 10:
                            speed = .05;
                            break;
                        case 5:
                            speed = 1.0;
                            break;
                    }
                }
            } else {
                keyA = false;
            }

            if (gamepad1.b) {
                if (keyB == false) {
                    keyB = !keyB;
                    invertDir = !invertDir;
                    if (invertDir) {
                        invDir = -1;
                    } else {
                        invDir = 1;
                    }
                }
            } else {
                invDir = 1;
            }

            leftFrontDrive.setPower(leftFrontPower * speed * invDir);
            leftBackDrive.setPower(leftBackPower * speed * invDir);
            rightFrontDrive.setPower(rightFrontPower * speed * invDir);
            rightBackDrive.setPower(rightBackPower * speed * invDir);

            if (gamepad1.y) {
                intake.setPower(.5);
            } else {
                intake.setPower(0.0);
            }

            //   if (gamepad1.x) {
            //       outtake.setPower(0.4);
            //   } else {
            //       outtake.setPower(0.0);
        }

    }
}





