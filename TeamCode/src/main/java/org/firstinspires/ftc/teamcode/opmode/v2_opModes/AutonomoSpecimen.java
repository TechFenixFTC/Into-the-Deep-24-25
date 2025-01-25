package org.firstinspires.ftc.teamcode.opmode.v2_opModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V2;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;


@Config
@Autonomous(name = "AutonomoSpecimen", group = "Autonomous")
public class AutonomoSpecimen extends LinearOpMode {
    V5 robot;

    @Override
    public void runOpMode()  {

        Pose2d initialPose = new Pose2d(-32, -61, Math.toRadians(180));
        robot = new V5(hardwareMap,telemetry);
        robot.md.pose = initialPose;

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        sample1(),
                        sample2(),
                        sample3(),
                        sample4(),
                        irParaCasa()



                )

        );

    }


    public Action goToDeposit(){
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(51, 54, Math.toRadians(-135)), Math.toRadians(45))
                        .build()
        );
    }
    public Action deposit() {
        return new SequentialAction(
                new ParallelAction(
                        goToDeposit()
                        //acções simultaneas para depositar
                )
        );

    }
    public Action intake(){
        return new SequentialAction(

                new ParallelAction(
                        new SequentialAction(

                        )

                )
        );
    }

    public Action goToSample2() {
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(47.5, 50, Math.toRadians(-90)), Math.toRadians(-90))
                        .build()
        );
    }
    public Action goToSample3() {
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(58, 50, Math.toRadians(-90)), Math.toRadians(-45))
                        .build()
        );
    }
    public Action goToSample4(){
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(46, 48, Math.toRadians(-45)), Math.toRadians(180))
                        .build()
        );
    }

    public Action sample1() {
        return new SequentialAction(
                deposit()
        );
    }
    public Action sample2() {
        return new SequentialAction(
                goToSample2(),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.2).build(),
                intake(),
                deposit()
        );
    }
    public Action sample3() {
        return new SequentialAction(
                goToSample3(),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.3).build(),
                intake(),
                deposit()
        );
    }
    public Action sample4(){
        return new SequentialAction(
                goToSample4(),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.3).build(),
                intake(),
                deposit()

        );
    }

    public Action irParaCasa(){
        return new SequentialAction(
                new ParallelAction(
                        //mover lineares para estacionar
                        goToHouse()
                )


        );
    }
    public Action goToHouse() {
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        .setTangent(Math.toRadians(-135))
                        .splineToLinearHeading(new Pose2d(30, 10, Math.toRadians(180)), Math.toRadians(-135))
                        .build()
        );

    }
}