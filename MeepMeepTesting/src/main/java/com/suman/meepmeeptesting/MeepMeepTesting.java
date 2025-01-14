package com.suman.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    private static final Pose2d zeroPose = new Pose2d(0, 0, 0);

    private static final Vector2d px = new Vector2d(-20, 0);

    private static final Vector2d py = new Vector2d(0, -20);

    private static final Pose2d STARTING_POSE = new Pose2d(-36, -63, Math.toRadians(270));



    // Sample positions (adjust these based on your field measurements)

    private static final Vector2d SPECIMEN_DROP = new Vector2d(-5, -36);

    private static final Vector2d SAMPLE_1 = new Vector2d(-48, -40);

    private static final Vector2d SAMPLE_2 = new Vector2d(-58, -40);

    private static final Vector2d SAMPLE_3 = new Vector2d(-66, -40);

    private static final Vector2d BUCKET_POS = new Vector2d(-48, -50);

    private static final Vector2d PARK_POS = new Vector2d(-60, -12);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        //red
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-12, -60, Math.toRadians(90)))
                // Move to specimen drop

                .strafeTo(SPECIMEN_DROP)

                .waitSeconds(0.5)//time for claw



                // Move to first sample

                .strafeTo(SAMPLE_1)

                .waitSeconds(0.5)  // Time for intake



                // Move to bucket

                .lineToYLinearHeading(-50, Math.toRadians(225))

                .waitSeconds(0.5)  // Time for outtake



                // Move to second sample.
                .strafeToLinearHeading(SAMPLE_2,Math.toRadians(90))
                //.turn(Math.toRadians(225))//turn this into .lineToLinearHeading(SAMPLE_2, Math.toRadians(225)

                //I cant because .lineToLinearHeading isn't showing up for me

                //.strafeTo(SAMPLE_2)

                .waitSeconds(0.5)  // Time for intake

//

                // Back to bucket

                .strafeToLinearHeading(BUCKET_POS, Math.toRadians(225))//turn this into .lineToLinearHeading(BUCKET_POS, Math.toRadians(225)

                //I cant because .lineToLinearHeading isn't showing up for me

                //.turn(Math.toRadians(135))

                .waitSeconds(0.5)  // Time for outtake



                // Move to third sample

                .strafeToLinearHeading(SAMPLE_3,Math.toRadians(90))
                //.turn(Math.toRadians(225))//turn this into .lineToLinearHeading(SAMPLE_3, Math.toRadians(225)

                //I cant because .lineToLinearHeading isn't showing up for me

                //.strafeTo(SAMPLE_3)

                .waitSeconds(0.5)  // Time for intake



                // Back to bucket one last time

                .strafeToLinearHeading(BUCKET_POS,Math.toRadians(225))

                //.turn(Math.toRadians(135))

                .waitSeconds(0.5)  // Time for outtake



                // Park

                //.turn(Math.toRadians(225))

                .strafeToLinearHeading(new Vector2d(-36,-63),Math.toRadians(90))


//        //blue
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-23, 52, Math.toRadians(-90)))
//                .splineTo(new Vector2d(0,35),Math.toRadians(-90))
//                //wait ant put speciamskdfjas;dlfkja onto the cage
//                .waitSeconds(2)
////                .lineToYLinearHeading(38,Math.toRadians(90))
//                .splineTo(new Vector2d(-48,38),Math.toRadians(-90))
//                        .splineTo(new Vector2d(0,35),Math.toRadians(90))
//                .splineTo(new Vector2d(Vector2d(-37.5,25),Math.toRadians(0))
                //do intake
                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

