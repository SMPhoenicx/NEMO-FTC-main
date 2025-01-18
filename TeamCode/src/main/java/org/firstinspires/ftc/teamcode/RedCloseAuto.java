package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
@Config
@Autonomous(name = "Red Close Auto")
public class RedCloseAuto extends LinearOpMode {
    // Define your robot's starting position
    private static final Pose2d zeroPose = new Pose2d(0, 0, 0);
    private static final Vector2d px = new Vector2d(20, 0);
    private static final Vector2d py = new Vector2d(0, 20);
    private static final Pose2d STARTING_POSE = new Pose2d(12, -60, Math.toRadians(90));

    // Sample positions (adjust these based on your field measurements)
    private static final Vector2d SPECIMEN_DROP = new Vector2d(9, -28);
    private static final Vector2d SAMPLE_1 = new Vector2d(24, -12);
    private static final Vector2d SAMPLE_2 = new Vector2d(0, -12);
    private static final Vector2d SAMPLE_3 = new Vector2d(-24, -12);
    private static final Vector2d BUCKET_POS = new Vector2d(0, -48);
    private static final Vector2d PARK_POS = new Vector2d(60, -12);
    private DcMotor rext = null;//ext are extension
    private DcMotor lext = null;
    private DcMotor rpivot = null; //pivot arm up and down
    private DcMotor lpivot = null;
    private CRServo servo1 = null; //intake
    private CRServo servo2 = null;
    private Servo sWrist = null; //wrist joint

    private static final int liftMax=1000;
    private static final int liftMin=-2900;
    private static final int pivotMax=1000;
    private static final int pivotMin=800;


    @Override
    public void runOpMode() throws InterruptedException {
        rext = hardwareMap.get(DcMotor.class, "rext");
        lext = hardwareMap.get(DcMotor.class, "lext");
        rpivot = hardwareMap.get(DcMotor.class, "rpivot");
        lpivot = hardwareMap.get(DcMotor.class, "lpivot");
        servo1 = hardwareMap.get(CRServo.class, "s1");
        servo2 = hardwareMap.get(CRServo.class, "s2");
        sWrist = hardwareMap.get(Servo.class, "sWrist");
        sWrist.setPosition(0);

        Lift lift = new Lift(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, STARTING_POSE);



        TrajectoryActionBuilder autoSequence = drive.actionBuilder(STARTING_POSE)
                .strafeTo(SPECIMEN_DROP)
                //wait ant put speciamskdfjas;dlfkja onto the cage
                .waitSeconds(2)
                .stopAndAdd(pivot.pivotDown())
                .stopAndAdd(lift.liftDown())
                .stopAndAdd(intake.intakeDown())

                .strafeTo(new Vector2d(30,-35))
                //.splineToSplineHeading(new Pose2d(0,-36,Math.toRadians(0)),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(36,-10),Math.toRadians(90))
                //.splineToConstantHeading(new Vector2d(35,-10),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(42,-10),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(42,-45),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(42,-10),Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(new Vector2d(52,-10),Math.toRadians(90)),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(52, -45), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(52,-10), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(new Vector2d(58,-10),Math.toRadians(90)),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(58, -45), Math.toRadians(90))
//move to pick up second block

                .splineTo(new Vector2d(56,-45), Math.toRadians(-90))

                .waitSeconds(0.5)//intake



                //move to drop off block w clip

                .splineToSplineHeading(new Pose2d(50,-45,Math.toRadians(180)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(3,-35,Math.toRadians(90)),Math.toRadians(90))

                .waitSeconds(0.5)//outtake



//move to pick up third block

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(50,-45,Math.toRadians(-90)),Math.toRadians(-90))
                .setReversed(false)

                .waitSeconds(0.5)//intake



//move to drop off block w clip

                .splineToSplineHeading(new Pose2d(40,-45,Math.toRadians(180)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(3,-35,Math.toRadians(90)),Math.toRadians(90))

                .waitSeconds(0.5)//outtake



//park

                .strafeTo(PARK_POS);

        waitForStart();

        if (isStopRequested()) return;

        // TODO: Set lift to specimen height
        //Actions.runBlocking(autoSequence);
        /**Actions.runBlocking(
                new SequentialAction(
                        autoSequence.build()
                )
        );**/
        // Throughout the sequence, you'll need to add your intake/lift controls
        // at the appropriate times
        Actions.runBlocking(
                new ParallelAction(
                        pivot.pivotUp(),
                        lift.liftUp(),
                        intake.intakeUp(),
                        //lift.liftDown(),
                        //pivot.pivotDown(),
                        //intake.intakeDown(),
                        //intake.intakeUp()
                        autoSequence.build()
                )
        );
        // Throughout the sequence, you'll need to add your intake/lift controls
        // at the appropriate times
    }
    public class Lift {

        public Lift(HardwareMap hardwareMap) {

            rext.setDirection(DcMotor.Direction.FORWARD);
            lext.setDirection(DcMotor.Direction.REVERSE);

            lext.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
            lext.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rext.setPower(-.5);
                    lext.setPower(-.5);
                    initialized = true;
                }
                double pos = lext.getCurrentPosition();
                packet.put("liftPos",pos);
                if (pos > liftMin) {
                    return true;
                } else {
                    rext.setPower(0);
                    lext.setPower(0);
                    return false;
                }
            }
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rext.setPower(1);
                    lext.setPower(1);
                    initialized = true;
                }
                double pos = lext.getCurrentPosition();
                packet.put("liftPos",pos);
                if (pos < liftMax) {
                    return true;
                } else {
                    rext.setPower(0);
                    lext.setPower(0);
                    return false;
                }
            }
        }

        public Action liftUp() {
            return new Lift.LiftUp();
        }

        public Action liftDown() {
            return new Lift.LiftDown();
        }
    }
    public class Pivot {

        public Pivot(HardwareMap hardwareMap) {

            rpivot.setDirection(DcMotor.Direction.FORWARD);
            lpivot.setDirection(DcMotor.Direction.FORWARD);

            lpivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
            lpivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
        }

        public class PivotUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rpivot.setPower(0.5);
                    lpivot.setPower(0.5);
                    initialized = true;
                }
                double pos = lpivot.getCurrentPosition();
                packet.put("pivotPos", pos);
                if (pos < pivotMax) {
                    return true;
                } else {
                    rpivot.setPower(0);
                    lpivot.setPower(0);
                    return false;
                }
            }
        }

        public class PivotDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    rpivot.setPower(-0.5);
                    lpivot.setPower(-0.5);
                    initialized = true;
                }
                double pos = lpivot.getCurrentPosition();
                packet.put("pivotPos", pos);
                if (pos > pivotMin) {
                    return true;
                } else {
                    rpivot.setPower(0);
                    lpivot.setPower(0);
                    return false;
                }
            }
        }

        public Action pivotUp() {
            return new Pivot.PivotUp();
        }

        public Action pivotDown() {
            return new Pivot.PivotDown();
        }
    }
    public class Intake {//dont know how to implement intake on and off, probably like.wait but doesn twokr
        //unless in the runOpMode soooo idk

        public Intake(HardwareMap hardwareMap) {

            servo1.setDirection(DcMotorSimple.Direction.FORWARD);
            servo2.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class IntakeUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    servo1.setPower(0.5);
                    servo2.setPower(0.5);
                    initialized = true;
                    resetRuntime();
                }
                if (getRuntime() < 1) {
                    return true;
                } else {
                    servo1.setPower(0);
                    servo2.setPower(0);
                    return false;
                }
            }
        }

        public class IntakeDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    servo1.setPower(-1);
                    servo2.setPower(-1);
                    initialized = true;
                    resetRuntime();
                }
                if (getRuntime() < 1) {
                    return true;
                } else {
                    servo1.setPower(0);
                    servo2.setPower(0);
                    return false;
                }
            }
        }

        public Action intakeUp() {
            return new Intake.IntakeUp();
        }

        public Action intakeDown() {
            return new Intake.IntakeDown();
        }
    }
}