package com.suman.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    private static final Pose2d zeroPose = new Pose2d(0, 0, 0);

    private static final Vector2d px = new Vector2d(20, 0);

    private static final Vector2d py = new Vector2d(0, 20);

    private static final Pose2d STARTING_POSE = new Pose2d(36, 63, Math.toRadians(270));



    // Sample positions (adjust these based on your field measurements)

    private static final Vector2d SPECIMEN_DROP = new Vector2d(5, 36);

    private static final Vector2d SAMPLE_1 = new Vector2d(48, 40);

    private static final Vector2d SAMPLE_2 = new Vector2d(58, 40);

    private static final Vector2d SAMPLE_3 = new Vector2d(68, 40);

    private static final Vector2d BUCKET_POS = new Vector2d(48, 50);

    private static final Vector2d PARK_POS = new Vector2d(60, -12);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        //red
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12, -60, Math.toRadians(90)))
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
                .splineToLinearHeading(new Pose2d(new Vector2d(60,-10),Math.toRadians(90)),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(60, -45), Math.toRadians(90))
//move to pick up second block

                .strafeTo(new Vector2d(56,-55))

                .waitSeconds(0.5)//intake



//move to drop off block w clip

                .turn( Math.toRadians(180))//turn into .lineLinearHeading()

                .strafeTo(new Vector2d(3,-35))

                .waitSeconds(0.5)//outtake



//move to pick up third block

                .turn( Math.toRadians(180))

                .strafeTo(new Vector2d(65,-55))

                .waitSeconds(0.5)//intake



//move to drop off block w clip

                .turn( Math.toRadians(180))//turn into .lineLinearHeading()

                .strafeTo(new Vector2d(3,-35))

                .waitSeconds(0.5)//outtake



//park

                .strafeTo(PARK_POS)


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

