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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "Red Close Auto")
public class RedCloseAuto extends LinearOpMode {
    // Define your robot's starting position
    private static final Pose2d zeroPose = new Pose2d(0, 0, 0);
    private static final Vector2d px = new Vector2d(20, 0);
    private static final Vector2d py = new Vector2d(0, 20);
    private static final Pose2d STARTING_POSE = new Pose2d(23, -52, Math.toRadians(90));

    // Sample positions (adjust these based on your field measurements)
    private static final Vector2d SPECIMEN_DROP = new Vector2d(0, -36);
    private static final Vector2d SAMPLE_1 = new Vector2d(24, -12);
    private static final Vector2d SAMPLE_2 = new Vector2d(0, -12);
    private static final Vector2d SAMPLE_3 = new Vector2d(-24, -12);
    private static final Vector2d BUCKET_POS = new Vector2d(0, -48);
    private static final Vector2d PARK_POS = new Vector2d(60, -12);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, STARTING_POSE);

        TrajectoryActionBuilder autoSequence = drive.actionBuilder(STARTING_POSE)
                .splineTo(new Vector2d(0,-35),Math.toRadians(90))
                //wait ant put speciamskdfjas;dlfkja onto the cage
                .waitSeconds(2)

                .lineToYLinearHeading(-38,Math.toRadians(-90))
                .splineTo(new Vector2d(35,-38),Math.toRadians(90))
                .splineTo(new Vector2d(35,-10),Math.toRadians(90))
                .splineTo(new Vector2d(43,-10),Math.toRadians(-90))
                .splineTo(new Vector2d(43,-55),Math.toRadians(-90))
                .lineToYLinearHeading(-10, Math.toRadians(-90))
                .splineTo(new Vector2d(56,-10),Math.toRadians(-90))
                .splineTo(new Vector2d(56, -55), Math.toRadians(-90))
                .lineToYLinearHeading(-10, Math.toRadians(90))
                .splineTo(new Vector2d(65,-10),Math.toRadians(-90))
                .splineTo(new Vector2d(65, -55), Math.toRadians(-90));

        waitForStart();

        if (isStopRequested()) return;

        // TODO: Set lift to specimen height
        //Actions.runBlocking(autoSequence);
        Actions.runBlocking(
                new SequentialAction(
                        autoSequence.build()
                )
        );
        // Throughout the sequence, you'll need to add your intake/lift controls
        // at the appropriate times
    }
}