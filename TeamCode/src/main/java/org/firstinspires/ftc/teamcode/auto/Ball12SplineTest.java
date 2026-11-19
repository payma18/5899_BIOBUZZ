package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
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

@Autonomous(name = "12 ball spline test", group = "Autonomous")
public class Ball12SplineTest extends LinearOpMode{

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
        public class Spit3500 implements Action {
            private boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (spitCount > 105) {
                    spitCount = 0;
                }
                if (spitCount > 15) {
                    upperServo.setPosition(0.0);
                }
                sleep(50);
                spitCount++;
                return spitCount <= 105; // true reruns action
            }
        }
        public Action spit3500() { return new Spit3500(); }

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

        Pose2d poseInit = new Pose2d(-63, 12, Math.toRadians(0));
        Pose2d poseInitptone = new Pose2d(-56, 12, Math.toRadians(0));
        Pose2d pose2 = new Pose2d(-61, 64, Math.toRadians(180));
        Pose2d pose3 = new Pose2d(56, 12, Math.toRadians(180));
        Pose2d pose4 = new Pose2d(58, 24, Math.toRadians(0));
        Pose2d pose5 = new Pose2d(56, 12, Math.toRadians(180));
        Pose2d pose6 = new Pose2d(48, 63, Math.toRadians(270));

        MecanumDrive drive = new MecanumDrive(hardwareMap, poseInit);
        outtakeMotor outtake = new outtakeMotor(hardwareMap);
        UpperTransfer transfer = new UpperTransfer(hardwareMap);
        Intaker intake = new Intaker(hardwareMap);

        TrajectoryActionBuilder one = drive.actionBuilder(poseInit)
                .splineToConstantHeading(new Vector2d(-56, 12), Math.toRadians(0));

        TrajectoryActionBuilder onepttwo = drive.actionBuilder(poseInitptone)
                .splineToLinearHeading(new Pose2d(-61, 64, Math.toRadians(180)), Math.toRadians(135));

        TrajectoryActionBuilder two = drive.actionBuilder(pose2)
                .splineToConstantHeading(new Vector2d(-30, 50), Math.toRadians(-20))
                .splineToSplineHeading(new Pose2d(56, 12, Math.toRadians(180)), Math.toRadians(-70));

        TrajectoryActionBuilder three = drive.actionBuilder(pose3)
                .splineToLinearHeading(new Pose2d(50, 18, Math.toRadians(0)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(58, 24), Math.toRadians(0));

        TrajectoryActionBuilder four = drive.actionBuilder(pose4)
                .splineToConstantHeading(new Vector2d(46, 18), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(56, 12, Math.toRadians(180)), Math.toRadians(0));

        TrajectoryActionBuilder five = drive.actionBuilder(pose5)
                .splineToLinearHeading(new Pose2d(48, 62, Math.toRadians(270)), Math.toRadians(90));


        // actions that need to happen on init

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        outtake.hold(),
                        new SequentialAction(
                                intake.intaking(),
                                one.build(),
                                transfer.spit3500(),
                                onepttwo.build(),
                                two.build(),
                                transfer.spit3500(),
                                three.build(),
                                four.build(),
                                transfer.spit3500(),
                                five.build()
                        )
                )
        );
    }
}
