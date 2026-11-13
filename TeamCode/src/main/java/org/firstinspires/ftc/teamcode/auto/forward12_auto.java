package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.old_robot_stuff.ShooterControl;

@Autonomous(name = "forward 12", group = "Autonomous")
public class forward12_auto extends LinearOpMode{

    double quarter = 90; // "90 degrees" / right angle turn
    double tile = 24; // "24 inches" / one tile
    private VoltageSensor battery;
    private ShooterControl flywheel;

    public class Intaker {
        private DcMotor intakeMotor;
        private Servo lowerServo;
        public Intaker(HardwareMap hwMap) {
            intakeMotor = hwMap.get(DcMotor.class, "intake");
            lowerServo = hwMap.get(Servo.class, "lower");
        }
        public class Intaking implements Action {
            private boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeMotor.setPower(-1.0);
                lowerServo.setPosition(0.0);
                return false; // true reruns action
            }
        }
        public Action intaking(){
            return new Intaking();
        }

        public class StopIntaking implements Action {
            private boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeMotor.setPower(0.0);
                lowerServo.setPosition(0.5);
                return false; // true reruns action
            }
        }
        public Action stopIntaking(){
            return new StopIntaking();
        }
    }
    public class UpperTransfer {
        private Servo upperServo;
        private int spitCount = 0;
        public UpperTransfer(HardwareMap hwMap) {
            upperServo = hwMap.get(Servo.class, "upper");
        }
        public class Spit5000 implements Action {
            private boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (spitCount > 150) {
                    spitCount = 0;
                }
                if (spitCount > 30) {
                    upperServo.setPosition(0.25);
                }
                sleep(50);
                spitCount++;
                return spitCount <= 150; // true reruns action
            }
        }
        public Action spit5000() { return new Spit5000(); }

        public class Spin implements Action {
            private boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                upperServo.setPosition(1.0);
                return false; // true reruns action
            }
        }
        public Action spin(){
            return new Spin();
        }

        public class StopSpinning implements Action {
            private boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                upperServo.setPosition(0.5);
                return false; // true reruns action
            }
        }
        public Action stopSpinning(){
            return new StopSpinning();
        }
    }
    public class outtakeMotor {
        private DcMotorEx shooter;
        double power = 0;
        private final ElapsedTime timer = new ElapsedTime();
        double dt = timer.seconds();
        double maxStep = 2;
        private boolean started = false;

        public outtakeMotor(HardwareMap hwMap) {
            shooter = hwMap.get(DcMotorEx.class, "flywheel");
            shooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            shooter.setDirection(DcMotorEx.Direction.FORWARD);
        }

        public class FireUp implements Action {

            private boolean started = false;
            private final ElapsedTime t = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started){t.reset(); started=true;}
                double dt = t.seconds();
                power += dt * 100;
                power = Range.clip(power, 0, 1.0);
                shooter.setPower(power);
                t.reset();
                return power < 0.9; // true reruns action
            }
        }
        public Action fireUp(){
            return new FireUp();
        }

        public class Hold implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                flywheel.setBatteryVoltage(battery.getVoltage());
                flywheel.setvoltCorr(1.5);
                flywheel.setKf(0.00083);
                flywheel.setKp(0.009);
                flywheel.setKi(0);
                flywheel.setKd(0.0009);
                flywheel.setTargetRPM(690); // // // // TODO: find a value for this
                flywheel.flywheelHold();
                return true; // true reruns action
            }
        }
        public Action hold(){
            return new Hold();
        }

        public class Stop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //shooter.setPower(0);
                flywheel.setTargetRPM(0);
                flywheel.flywheelHold();
                return false; // true reruns action
            }
        }
        public Action stop(){
            return new Stop();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        battery = hardwareMap.voltageSensor.iterator().next();

        flywheel = new ShooterControl(hardwareMap);

        Pose2d poseInit = new Pose2d(0, 0, Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, poseInit);
        outtakeMotor outtake = new outtakeMotor(hardwareMap);
        UpperTransfer transfer = new UpperTransfer(hardwareMap);
        Intaker intake = new Intaker(hardwareMap);

        TrajectoryActionBuilder one = drive.actionBuilder(poseInit)
                .strafeToLinearHeading(new Vector2d(12, 0), Math.toRadians(0));//, new TranslationalVelConstraint(10));

        // actions that need to happen on init

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                one.build()
        );
    }
}
