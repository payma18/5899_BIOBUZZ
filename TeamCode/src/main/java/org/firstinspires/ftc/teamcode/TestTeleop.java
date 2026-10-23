package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="TestTeleop", group="Linear OpMode")
public class TestTeleop extends LinearOpMode {
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotor intake;
    VoltageSensor battery;
    enum FlywheelState {RAMP, HOLD, DOWN};
    FlywheelState state = FlywheelState.RAMP;
    private DcMotorEx flywheel;
    final double TICKS_PER_REV = 112;

    ElapsedTime timer = new ElapsedTime();

    ElapsedTime timerTop = new ElapsedTime();

    double batteryVoltage;

    double targetRPM = 500; // TODO: SET
    double rampingRPM = 0;
    double maxRPM = 5000; //TODO: find a value
    double currentRPM, smoothedRPM; // currentRPM from getVelocity will always have noise, so smoothedRPM will make it a little more steady
    double correctPow, smoothedPow = 1;

    double voltCorr =1.5;
    double Kp = 0.009;
    double Kf = 0.00083;//0.000749;
    double Kd = 0.0009; //TODO: find;
    double Ki = 0, integralSum, derivative; // prob won't use ki kd but have in case
    double error, lastError;

    boolean firstLoop = true;

    public double rpmToPower(double targetRPM, double dt)  {

        //dt = Range.clip(timer.seconds(), 0, 0.05);

        // get encoder pos
        currentRPM = flywheel.getVelocity() * 60 / TICKS_PER_REV;

        if (firstLoop) { // need a working initial value for smoothedRPM
            smoothedRPM = currentRPM;
            firstLoop = false;
        }

        smoothedRPM = (0.90 * smoothedRPM + 0.1 * currentRPM);

        error = targetRPM - smoothedRPM;

        if (Math.abs(error) < 200) {integralSum +=  (error * dt);} // prevent spiking/windup

        if (dt >= 1e-4) {derivative = (error - lastError) / dt;} // skip if dt is too small because then derivative will be huge

        //correctPow = (Kp * error) + (Ki * integralSum) + (Kd * derivative) + (Kf * targetRPM / batteryVoltage); //13.6 ish at beginning
        correctPow = (Kp * error) + (Ki * integralSum) + (Kd * derivative) + (Kf * targetRPM * 12.6 / (Math.pow(batteryVoltage, voltCorr)));

        correctPow = Range.clip(correctPow,-1,1);
        smoothedPow = (smoothedPow * 0.85) + (correctPow * 0.15);

        lastError = error;

        return smoothedPow;

    }
    public void flywheelUpdate() {

        double dt = timer.seconds();
        dt = Range.clip(dt, 0.005, 0.05);
        //double dt = Math.max(dt, 0.005);

        correctPow = rpmToPower(rampingRPM, dt);
        flywheel.setPower(correctPow);

        timer.reset();

    }
    public void flywheelHold(){

        batteryVoltage = battery.getVoltage();

        //double dt = Range.clip(timer.seconds(), 0, 0.05);
        double dt = timer.seconds();
        dt = Range.clip(dt, 0.005, 0.05);

        correctPow = rpmToPower(targetRPM, dt);
        flywheel.setPower(correctPow);

        timer.reset();

    }
    public void flywheelStop(){

        flywheel.setPower(0);

        timer.reset();

    }

    public void runOpMode() {

        battery = hardwareMap.voltageSensor.iterator().next();
        flywheel = hardwareMap.get(DcMotorEx.class, "test");
        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setDirection(DcMotorEx.Direction.FORWARD); // pick the correct one

        timer.reset();

        MultipleTelemetry mtelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();




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

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);



        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) targetRPM++;
            if (gamepad1.b) targetRPM--;
            flywheelHold();
            mtelemetry.addData("target", targetRPM);
            mtelemetry.addData("RPM", smoothedRPM);
            mtelemetry.addData("Power", correctPow);
            mtelemetry.addData("Used Power", smoothedPow);
            mtelemetry.update();

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

            // half speed controls
            if (gamepad1.dpad_up) {
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

            //inverted direction controls
            if (gamepad1.dpad_down) {
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
        }
    }
}
