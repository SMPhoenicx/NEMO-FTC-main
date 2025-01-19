package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "Sample Specimen Auto")
public class PrototypePlaceAuto extends LinearOpMode {
    // Define your robot's starting position
    private static final Pose2d zeroPose = new Pose2d(0, 0, 0);
    private static final Vector2d px = new Vector2d(20, 0);
    private static final Vector2d py = new Vector2d(0, 20);
    private static final Pose2d STARTING_POSE = new Pose2d(-36, -63, Math.toRadians(90));

    // Sample positions (adjust these based on your field measurements)
    private static final Vector2d SPECIMEN_DROP = new Vector2d(0, -36);
    private static final Vector2d SAMPLE_1 = new Vector2d(24, -12);
    private static final Vector2d SAMPLE_2 = new Vector2d(0, -12);
    private static final Vector2d SAMPLE_3 = new Vector2d(-24, -12);
    private static final Vector2d BUCKET_POS = new Vector2d(0, -48);
    private static final Vector2d PARK_POS = new Vector2d(55, -55);






    // Initializing motors for lift and intake
    private DcMotor rext = null;//ext are extension
    private DcMotor lext = null;
    private DcMotor rpivot = null; //pivot arm up and down
    private DcMotor lpivot = null;
    private CRServo servo1 = null; //intake
    private CRServo servo2 = null;
    private Servo sWrist = null; //wrist joint

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, zeroPose);
        rext = hardwareMap.get(DcMotor.class, "rext");
        lext = hardwareMap.get(DcMotor.class, "lext");
        rpivot = hardwareMap.get(DcMotor.class, "rpivot");
        lpivot = hardwareMap.get(DcMotor.class, "lpivot");
        servo1 = hardwareMap.get(CRServo.class, "s1");
        servo2 = hardwareMap.get(CRServo.class, "s2");
        sWrist = hardwareMap.get(Servo.class, "sWrist");

        rext.setDirection(DcMotor.Direction.FORWARD);
        lext.setDirection(DcMotor.Direction.REVERSE);
        rpivot.setDirection(DcMotor.Direction.FORWARD);
        lpivot.setDirection(DcMotor.Direction.FORWARD);
        servo1.setDirection(DcMotorSimple.Direction.FORWARD);
        servo2.setDirection(DcMotorSimple.Direction.FORWARD);
        sWrist.setDirection(Servo.Direction.FORWARD);
        sWrist.setPosition(0.5);
        lext.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        lext.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
        lpivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        lpivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done

        // Build the entire action sequence
        TrajectoryActionBuilder testSequence = drive.actionBuilder(zeroPose)
                .strafeTo(px)
                .waitSeconds(1)
                .strafeTo(py)
                .waitSeconds(1)
                .turn(Math.toRadians(90));

        TrajectoryActionBuilder autoSequence = drive.actionBuilder(STARTING_POSE)
                // Move to specimen drop
                .strafeTo(SPECIMEN_DROP)
                .turn(Math.toRadians(90))

                // Move to first sample
                .strafeTo(SAMPLE_1)
                .turn(Math.toRadians(0))
                .waitSeconds(0.5)  // Time for intake

                // Move to bucket
                .strafeTo(BUCKET_POS)
                .turn(Math.toRadians(90))
                .waitSeconds(0.5)  // Time for outtake

                // Move to second sample
                .strafeTo(SAMPLE_2)
                .turn(Math.toRadians(0))
                .waitSeconds(0.5)  // Time for intake

                // Back to bucket
                .strafeTo(BUCKET_POS)
                .turn(Math.toRadians(90))
                .waitSeconds(0.5)  // Time for outtake

                // Move to third sample
                .strafeTo(SAMPLE_3)
                .turn(Math.toRadians(0))
                .waitSeconds(0.5)  // Time for intake

                // Back to bucket one last time
                .strafeTo(BUCKET_POS)
                .turn(Math.toRadians(90))
                .waitSeconds(0.5)  // Time for outtake

                // Park
                .strafeTo(PARK_POS)
                .turn(Math.toRadians(0));

        waitForStart();

        if (isStopRequested()) return;

        // TODO: Set lift to specimen height
        //Actions.runBlocking(autoSequence);
        Actions.runBlocking(
                new SequentialAction(
                        testSequence.build()
                )
        );
        // Throughout the sequence, you'll need to add your intake/lift controls
        // at the appropriate times
    }
}
