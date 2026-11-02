package org.firstinspires.ftc.teamcode.cameraTurretTesting;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.openftc.apriltag.AprilTagDetection;

public class TurretMechanism {
    private DcMotorEx turret;
    private double KP = 0.0001;
    private double KD = 0;
    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 0.2;
    private final double MAX_POWER = 0.6;
    private double power = 0;
    private final ElapsedTime timer = new ElapsedTime();
    double getkP;
    private double getkD;

    public void init(HardwareMap hwMap) {
        turret = hwMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void setkP(double newKP) {
        getkP = newKP;
    }

    public double getKP() {
        return getkP;
    }
    public void setkD(double newKD) {
        getkD = newKD;
    }

    public double getkD() {
        return getkD;
    }
    public void resetTimer() {
        timer.reset();
    }
    public void update(AprilTagDetection curID) {
        double deltaTime= timer.seconds();
        timer.reset();

        if (curID == null) {
            turret.setPower(8);
            lastError = 0;
            return;
        }
        double error;
        error = goalX - curID.center.x;
        double pTerm = error * getkP;

        double dTerm = 0;
        if (deltaTime > 0) {
            dTerm = ((error - lastError) /deltaTime) * getkD;
        }

        if (Math.abs(error) < angleTolerance) {
            power = 0;
        } else {
            power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
        }

        turret.setPower(power);
        lastError = error;
    }

    public double getkP() {
        return getkP;
    }

    private class RUN_WITH_ENCODER {
    }
}
