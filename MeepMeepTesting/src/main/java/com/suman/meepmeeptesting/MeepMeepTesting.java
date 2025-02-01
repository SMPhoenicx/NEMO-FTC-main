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
                .waitSeconds(7)
                .strafeTo(new Vector2d(33,-35))
                .splineToConstantHeading(new Vector2d(33,-34),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(33,-10),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(42,-10),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(42,-45),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(42,-10),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(52,-10),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(52, -45), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(52,-10), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(57,-10),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(57, -45), Math.toRadians(90))
                .splineTo(new Vector2d(54, -45), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(50, -53, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(3, -35, Math.toRadians(90)), Math.toRadians(90))
                .strafeTo(SPECIMEN_DROP)

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

