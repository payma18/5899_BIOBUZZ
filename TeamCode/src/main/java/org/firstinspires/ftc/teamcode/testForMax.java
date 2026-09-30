package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="test fpr max", group="Linear OpMode")
public class testForMax extends LinearOpMode {
    DcMotor test = null;

    public void runOpMode(){

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        test = hardwareMap.get(DcMotor.class,"test");

        waitForStart();
        /*
        double testUp = gamepad1.right_trigger;
        double testDown = -gamepad1.left_trigger;
        double testMiddle = gamepad1.left_stick_x;
        double testPower = testMiddle;
        double testMore = testMiddle + testUp;
        double testLess = testMiddle - testDown;
        double Max = testPower;
        if (testMore > Max){
            Max = testMore;
        }
        if (testLess < Max){
            Max = testLess;
        }
        testPower/= Max;
        test.setPower(testPower);
        */
        while(opModeIsActive()){
            test.setPower(gamepad1.right_stick_y * 0.250);
        }
    }
    public void buildTelemetry(){
        //myOpMode.telemetry.addData("testPower",test.getPower());
        //
        }
}
