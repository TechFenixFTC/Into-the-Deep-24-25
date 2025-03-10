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
@Autonomous(name = "AutonomoSample", group = "Autonomous")
public class AutonomoSample extends LinearOpMode {
    V5 robot;
    public  static double InitialX = 35, InitialY = 62,
            DepositX = 51 , DepositY = 54,
            Goto2X = 47.5 , Goto2Y = 50,
            Goto3X = 58 , Goto3Y = 50,
            Goto4X = 46, Goto4y = 48,
            EndX = 30 ,EndY = 10;

    @Override
    public void runOpMode()  {

        Pose2d initialPose = new Pose2d(InitialX, InitialY, Math.toRadians(180));
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
                        .splineToLinearHeading(new Pose2d(DepositX, DepositY, Math.toRadians(-135)), Math.toRadians(45))
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
    public Action ReadToIntake(){
        return new SequentialAction();
    }

    public Action goToSample2() {
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(Goto2X, Goto2Y, Math.toRadians(-90)), Math.toRadians(-90))
                        .build()
        );
    }
    public Action goToSample3() {
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        .setTangent(Math.toRadians(-45))
                        .splineToLinearHeading(new Pose2d(Goto3X, Goto3Y, Math.toRadians(-90)), Math.toRadians(-45))
                        .build()
        );
    }
    public Action goToSample4(){
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(Goto4X, Goto4y, Math.toRadians(-45)), Math.toRadians(180))
                        .build()
        );
    }

    public Action sample1(){
        return new SequentialAction(
                deposit()
        );
    }
    public Action sample2(){
        return new SequentialAction(
                new ParallelAction(
                        goToSample2(),
                        ReadToIntake()
                ),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.2).build(),
                intake(),
                deposit()
        );
    }
    public Action sample3(){
        return new SequentialAction(
                new ParallelAction(
                        goToSample3(),
                        ReadToIntake()
                ),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.3).build(),
                intake(),
                deposit()
        );
    }
    public Action sample4(){
        return new SequentialAction(
                new ParallelAction(
                        goToSample4(),
                        ReadToIntake()
                ),
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
    public Action goToHouse(){
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose)
                        .setTangent(Math.toRadians(-135))
                        .splineToLinearHeading(new Pose2d(EndX, EndY, Math.toRadians(180)), Math.toRadians(-135))
                        .build()
        );

    }
}