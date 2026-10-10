package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Driving Dynamics Testing", group="Linear OpMode")
public class DrivingDynamicsTesting extends LinearOpMode {

    private MecanumDrive drive;
    private IMU imu;

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    ElapsedTime stateTimer = new ElapsedTime();

    private void driveFieldRelative(double forward, double right, double rotate) {

        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate);
    }

    public void drive(double forward, double right, double rotate) {

        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        leftFrontDrive.setPower(frontLeftPower  / maxPower);
        rightFrontDrive.setPower(frontRightPower / maxPower);
        leftBackDrive.setPower(backLeftPower  / maxPower);
        rightBackDrive.setPower(backRightPower / maxPower);
    }

    @Override
    public void runOpMode() {

        ElapsedTime timer = new ElapsedTime();

        final int CYCLE_MS = 50;

        double speed = 1.0;   // used to manage half speed, defaults to full speed
        boolean invertDir = false;  // used for inverted direction prompts
        int invDir = 1;    // used to activate inverted direction
        boolean keyA = false, keyB = false;    // used for toggle keys

        double C_LATERAL, C_AXIAL, C_YAW;
        boolean C_FR = false, C_FREEZE = false, C_PREV_FR, C_PREV_FREEZE, C_H_RESET;

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        boolean driveRelative = false, freezePos = false;

        TrajectoryActionBuilder hold = drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(-24, -24), Math.toRadians(0));//, new TranslationalVelConstraint(10));

        while (opModeIsActive()) {

            drive.updatePoseEstimate();
            double heady = drive.localizer.getPose().heading.toDouble();

            C_AXIAL = Math.pow(gamepad1.left_stick_y, 1);
            C_LATERAL = Math.pow(gamepad1.left_stick_x, 1);
            C_YAW = Math.pow(gamepad1.right_stick_x, 1);
            C_PREV_FR = C_FR;
            C_FR = gamepad1.a;
            C_PREV_FREEZE = C_FREEZE;
            C_FREEZE = gamepad1.b;
            C_H_RESET = gamepad1.x;

            if (C_H_RESET) {
                imu.resetYaw();
            }

            if (C_FR && !C_PREV_FR) {
                driveRelative = !driveRelative;
            }

            Action action = hold.build();
            if (C_FREEZE && !C_PREV_FREEZE) {
                drive.localizer.setPose(drive.localizer.getPose());
                hold = drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(0));//, new TranslationalVelConstraint(10));
                action = hold.build();
            }
            if (C_FREEZE) {
                FtcDashboard dash = FtcDashboard.getInstance();
                Canvas previewCanvas = new Canvas();
                action.preview(previewCanvas);
                if (!Thread.currentThread().isInterrupted()) {
                    TelemetryPacket packet = new TelemetryPacket();
                    packet.fieldOverlay().getOperations().addAll(previewCanvas.getOperations());
                    dash.sendTelemetryPacket(packet);
                }
            }

            if (!C_FREEZE) {
                if (driveRelative) {
                    driveFieldRelative(C_AXIAL * .5, C_LATERAL * .5, C_YAW * .5);
                } else {
                    drive(C_AXIAL * .5, C_LATERAL * .5, C_YAW * .5);
                }
            }

            telemetry.update();

            sleep(CYCLE_MS);

        }
    }
}