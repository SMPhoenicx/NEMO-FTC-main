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
                .splineTo(new Vector2d(12,-35),Math.toRadians(90))
                //wait ant put speciamskdfjas;dlfkja onto the cage
                .waitSeconds(2)

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
        Actions.runBlocking(
                new SequentialAction(
                        autoSequence.build()
                )
        );
        // Throughout the sequence, you'll need to add your intake/lift controls
        // at the appropriate times
    }
}