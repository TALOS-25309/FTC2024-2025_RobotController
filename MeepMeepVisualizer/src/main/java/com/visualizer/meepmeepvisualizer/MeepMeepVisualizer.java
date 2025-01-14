package com.visualizer.meepmeepvisualizer;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepVisualizer {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Vector2d robotPose = new Vector2d(-40, 40);
        Vector2d BASKET_POSITION = new Vector2d(-72, -72);

        double BASKET_RADIUS = 24;

        double dx = BASKET_POSITION.x - robotPose.x;
        double dy = BASKET_POSITION.y - robotPose.y;
        double angle = Math.atan2(dy, dx);

        Vector2d target = new Vector2d(
                BASKET_POSITION.x - Math.cos(angle) * BASKET_RADIUS,
                BASKET_POSITION.y - Math.sin(angle) * BASKET_RADIUS);

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(robotPose.x, robotPose.y, Math.toRadians(0)))
                .splineTo(target, angle)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}