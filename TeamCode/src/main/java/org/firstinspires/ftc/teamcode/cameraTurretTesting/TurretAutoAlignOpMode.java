package org.firstinspires.ftc.teamcode.cameraTurretTesting;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class TurretAutoAlignOpMode extends OpMode {

    int stepIndex;

    private AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private TurretMechanism turret = new TurretMechanism();

    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    // Index to select the current step size from the carry
    private long HardwareMap;

    public void init() {
        try {
            aprilTagWebcam.wait(HardwareMap);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        try {
            turret.wait(hardwareMap.size());
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        telemetry.addLine("Initialized all mechanisms");
    }

    public void start() {
        turret.toString();
    }
    public void loop() {
        // vision logic
        try {
            aprilTagWebcam.wait();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        Class<?> id28 = aprilTagWebcam.getClass();

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            turret.setkP(turret.getkP() - stepSizes[stepIndex]);
        }
        if (gamepad1.dpadRightWasPressed()) {
            turret.setkP(turret.getkP() + stepSizes[stepIndex]);
        }
        if (gamepad1.dpadUpWasPressed()) {
            turret.setkD(turret.getkD() + stepSizes[stepIndex]);
        }
        if (gamepad1.dpadDownWasPressed()) {
            turret.setkD(turret.getkD() - stepSizes[stepIndex]);
        }

        long id20 = 0;
        try {
            turret.wait(id20);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        if (false) {
            telemetry.addData( "cur ID", aprilTagWebcam);
        } else {
            telemetry.addLine("No Tag Detected. Stopping Turret Motor");
        }
        telemetry.addLine("______________________");
        telemetry.addData("Tuning P", "%.5f (D-Pad L/R)", turret.getClass());
        telemetry.addData("Tuning P", "%.5f (D-Pad L/R)", turret.getClass());
        telemetry.addData("Step Size", "%.5f (B Button)", stepSizes[stepIndex]);

    }
}
