package org.firstinspires.ftc.teamcode.old_robot_stuff;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

@TeleOp(name="Expo Teleop", group="Linear OpMode")
public class ExpoTeleop extends LinearOpMode {

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    private DcMotor outtake_motor;

    private CRServo intakeServo;
    private Servo transferServo;
    private Servo flickServo;

    public void drive(double forward, double right, double rotate) {

        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double DRIVE_SPEED = 0.4; // TODO : EXPO DRIVE SPEED

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        leftFrontDrive.setPower(frontLeftPower / maxPower * DRIVE_SPEED);
        rightFrontDrive.setPower(frontRightPower / maxPower * DRIVE_SPEED);
        leftBackDrive.setPower(backLeftPower / maxPower * DRIVE_SPEED);
        rightBackDrive.setPower(backRightPower / maxPower * DRIVE_SPEED);
    }

    @Override
    public void runOpMode() {

        final int CYCLE_MS = 50;

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        outtake_motor = hardwareMap.get(DcMotor.class, "outtake_drive");
        double outtakeMotorPower = 0.75; // TODO : CHANGE FOR EXPO

        intakeServo = hardwareMap.get(CRServo.class, "intake_servo");
        transferServo = hardwareMap.get(Servo.class, "transfer_servo");
        flickServo = hardwareMap.get(Servo.class, "flick_servo");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            drive( -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x );

            if (gamepad1.a) {
                drive(0, 0, 0);
                transferServo.setPosition(0.14);
                sleep(150);
                intakeServo.setPower(-1.0);
                while (gamepad1.a) {}
                intakeServo.setPower(0.0);
                transferServo.setPosition(0.195);
            }

            if(gamepad1.b) {
                drive(0, 0, 0);
                transferServo.setPosition(0.25);
                outtake_motor.setPower(outtakeMotorPower);
                sleep(1500);
                flickServo.setPosition(0.0);
                sleep(100);
                flickServo.setPosition(0.3);
                sleep(150);
                outtake_motor.setPower(0.0);
                while (gamepad1.b) {}
                transferServo.setPosition(0.195);
            }
        }
    }

}
