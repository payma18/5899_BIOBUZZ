package org.firstinspires.ftc.teamcode.testrigs;

import static java.lang.Math.ceil;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="FWRegTest", group="Linear OpMode")
public class FlywheelRegulationTest extends LinearOpMode {

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
        flywheel.setDirection(DcMotorEx.Direction.REVERSE); // pick the correct one

        timer.reset();

        MultipleTelemetry mtelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
        }
    }
}
