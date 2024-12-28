package org.firstinspires.ftc.teamcode.opmode.v2_opModes;
import static org.firstinspires.ftc.teamcode.agregadoras.V2.deposit_x;
import static org.firstinspires.ftc.teamcode.agregadoras.V2.deposit_y;
import static org.firstinspires.ftc.teamcode.agregadoras.V2.risky;

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

import org.firstinspires.ftc.teamcode.agregadoras.V2;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Config
@Autonomous(name = "AutonomoA5", group = "Autonomous")
public class AutonomoPertoDaBasket extends LinearOpMode {
    V2 robot;
    public static double sample2_X = -40 , sample2_y = -52, sample3_x = -50,sample3_y = -51, adicional_sample_3 = 3.5;
    public boolean sample1 = false;
    @Override
    public void runOpMode()  {

        Pose2d initialPose = new Pose2d(-32, -61, Math.toRadians(180));
        robot = new V2(hardwareMap, telemetry);
        robot.md.pose = initialPose;
        robot.intakeOutake.braco.autonomo = true;
        MecanumDrive.PARAMS.kV = 0.0005;

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        sample1(),
                        sample2(),
                        sample3(),
                        irParaCasa()
                        //robot.intakeOutake.linearVertical.ElevadorGoTo(2800),
                        //robot.intakeOutake.linearHorizontal.extenderNemTanto(0,0),
                        //robot.md.actionBuilder(robot.md.pose).waitSeconds(2).build(),
                        //robot.intakeOutake.linearHorizontal.recolher(0,0)


                )

        );

    }



    public Action deposit() {
        return new SequentialAction(
                robot.intakeOutake.garra.rotacionarGarraParaPosicaoParalela(),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.35).build(),
                robot.intakeOutake.garra.abrirGarra(),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.6).build(),
                robot.intakeOutake.braco.goToStored(0,0)

        );

    }
    public Action sample1() {
        return new SequentialAction(
                robot.goDepositRedBasket("strafe"),
                deposit()

        );
    }
    public Action goDepositSample3(String tipo) {
        this.robot.md.updatePoseEstimate();
        if (tipo.equals("strafe")) {
            if(risky) {
                return new SequentialAction(
                        new InstantAction(() -> {MecanumDrive.PARAMS.kV *= 5;}),
                        new InstantAction(() -> {MecanumDrive.PARAMS.kA /= 2;}),

                        new ParallelAction(
                                this.robot.intakeOutake.linearVertical.ElevadorGoTo(2800),

                                this.robot.intakeOutake.garra.rotacionarGarraParaPosicaoPerpendicular(),
                                this.robot.md.actionBuilder(this.robot.md.pose)
                                        .setTangent(Math.toRadians(90))
                                        .strafeToLinearHeading(new Vector2d(deposit_x,deposit_y), Math.toRadians(-135))

                                        .build()

                        ),
                        this.robot.intakeOutake.braco.goToBasketOutake(0,0),
                        new InstantAction(() -> {MecanumDrive.PARAMS.kV /= 5;}),
                        new InstantAction(() -> {MecanumDrive.PARAMS.kA *= 2;})

                );
            }
            return new SequentialAction(
                    new ParallelAction(
                            this.robot.intakeOutake.linearVertical.ElevadorGoTo(2800),
                            this.robot.intakeOutake.braco.goToBasketOutakeSample3(0, 0),
                            this.robot.intakeOutake.garra.rotacionarGarraParaPosicaoPerpendicular(),
                            this.robot.md.actionBuilder(this.robot.md.pose)
                                    .setTangent(Math.toRadians(90))
                                    .strafeToLinearHeading(new Vector2d(-47, -51.5), Math.toRadians(-135))

                                    .build()

                    ),
                    this.robot.intakeOutake.linearHorizontal.extenderNemTanto(0, 0)
            );
        }
        return new SequentialAction(
                new ParallelAction(
                        this.robot.intakeOutake.linearVertical.ElevadorGoTo(2800),

                        this.robot.intakeOutake.garra.rotacionarGarraParaPosicaoPerpendicular(),
                        this.robot.md.actionBuilder(this.robot.md.pose)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(-48, -45,  Math.toRadians(-135)), Math.toRadians(-135))

                                .build()

                ),
                this.robot.intakeOutake.braco.goToBasketOutake(0, 0)
        );
    }




    public Action goToSample2() {
        return new SequentialAction(
                new ParallelAction(
                        robot.md.actionBuilder(robot.md.pose)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(sample2_X, sample2_y, Math.toRadians(90)), Math.toRadians(90))
                                .build(),

                        robot.intakeOutake.garra.abrirGarra(),
                        robot.intakeOutake.linearVertical.ElevadorGoTo(680),
                        robot.intakeOutake.garra.rotacionarGarraParaPosicaoParalela(),
                        robot.intakeOutake.linearHorizontal.extender(0,0),
                        new SequentialAction(

                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.6).build(),
                                robot.intakeOutake.braco.goToIntakePositon(0, 0)
                        )



                )

        );
    }


    public Action intake(){
        return new SequentialAction(
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.6).build(),
                robot.intakeOutake.linearVertical.ElevadorGoTo(80),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                robot.intakeOutake.garra.fecharGarra(),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.4).build(),
                robot.intakeOutake.linearVertical.ElevadorGoTo(320),
                robot.intakeOutake.braco.goToBasketOutake(0, 0),
                new ParallelAction(
                        new SequentialAction(
                                robot.intakeOutake.braco.goTo150(0, 0),
                                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.1).build(),
                                robot.intakeOutake.braco.goToStored(0,0)
                        ),
                        robot.intakeOutake.linearHorizontal.recolher(0,0)
                )
        );
    }
    public Action sample2() {
        return new SequentialAction(
                goToSample2(),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.2).build(),

                intake(),

                robot.goDepositRedBasket("strafe"),
                new InstantAction(() -> {robot.intakeOutake.braco.autonomo = true;}),
                deposit()
        );
    }
    public Action goToSample3() {
        return new SequentialAction(
                new ParallelAction(
                        robot.md.actionBuilder(robot.md.pose)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(sample3_x, sample3_y,  Math.toRadians(90)), Math.toRadians(90))
                                .build(),
                        robot.intakeOutake.garra.abrirGarra(),
                        robot.intakeOutake.linearVertical.ElevadorGoTo(680),
                        robot.intakeOutake.garra.rotacionarGarraParaPosicaoParalela(),
                        robot.intakeOutake.linearHorizontal.extender(0,0),
                        robot.intakeOutake.linearHorizontal.extender(0,0),
                        new SequentialAction(

                                robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                                robot.intakeOutake.braco.goToIntakePositon(0, 0)
                        )


                )

        );
    }

    public Action sample3() {
        return new SequentialAction(
                goToSample3(),
                robot.md.actionBuilder(robot.md.pose).waitSeconds(0.3).build(),
                intake(),
                new InstantAction(() -> {
                deposit_y -= adicional_sample_3;
                }),
                robot.goDepositRedBasket("strafe"),
                new InstantAction(() -> {robot.intakeOutake.braco.autonomo = true;}),
                deposit()
        );
    }
    public Action irParaCasa()
    {
        return new SequentialAction(
                new ParallelAction(
                        robot.intakeOutake.braco.goToStored(0,0),
                        robot.intakeOutake.linearHorizontal.recolher(0,0)
                ),
                new ParallelAction(
                        robot.intakeOutake.linearVertical.ElevadorGoTo(0),
                        goToHouse()
                ),
                robot.intakeOutake.braco.goToPark(0, 0)

               // robot.intakeOutake.braco.goTotouchBar()
        );
    }
    public Action goToHouse() {
        return new SequentialAction(
                new InstantAction(() -> {MecanumDrive.PARAMS.kV *= 10;}),
                robot.md.actionBuilder(robot.md.pose)
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-8, -16,  Math.toRadians(0)), Math.toRadians(-20))
                        .build()
        );

    }
}