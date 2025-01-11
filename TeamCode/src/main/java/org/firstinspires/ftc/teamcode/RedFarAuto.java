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
@Autonomous(name = "Red Far Auto")
public class RedFarAuto extends LinearOpMode {
    private static final Pose2d zeroPose = new Pose2d(0, 0, 0);

    private static final Vector2d px = new Vector2d(20, 0);

    private static final Vector2d py = new Vector2d(0, 20);

    private static final Pose2d STARTING_POSE = new Pose2d(-12, -63, Math.toRadians(90));



    // Sample positions (adjust these based on your field measurements)

    private static final Vector2d SPECIMEN_DROP = new Vector2d(-5, -36);

    private static final Vector2d SAMPLE_1 = new Vector2d(-48, -40);

    private static final Vector2d SAMPLE_2 = new Vector2d(-58, -40);

    private static final Vector2d SAMPLE_3 = new Vector2d(-68, -40);

    private static final Vector2d BUCKET_POS = new Vector2d(-48, -50);

    private static final Vector2d PARK_POS = new Vector2d(60, -12);
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, STARTING_POSE);

        TrajectoryActionBuilder autoSequence = drive.actionBuilder(STARTING_POSE)
                // Move to specimen drop

                .strafeTo(SPECIMEN_DROP)

                .waitSeconds(0.5)//time for claw



                // Move to first sample

                .strafeTo(SAMPLE_1)

                .waitSeconds(0.5)  // Time for intake



                // Move to bucket

                .lineToYLinearHeading(-50, Math.toRadians(225))

                .waitSeconds(0.5)  // Time for outtake



                // Move to second sample

                .turn(Math.toRadians(235))//turn this into .lineToLinearHeading(SAMPLE_2, Math.toRadians(225)

                //I cant because .lineToLinearHeading isn't showing up for me

                .strafeTo(SAMPLE_2)

                .waitSeconds(0.5)  // Time for intake

//

                // Back to bucket

                .strafeTo(BUCKET_POS)//turn this into .lineToLinearHeading(BUCKET_POS, Math.toRadians(225)

                //I cant because .lineToLinearHeading isn't showing up for me

                .turn(Math.toRadians(135))

                .waitSeconds(0.5)  // Time for outtake


                // Park

                .turn(Math.toRadians(235 ))
                .strafeTo(new Vector2d(-40,0))
                .strafeTo((new Vector2d(-20,0)));

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