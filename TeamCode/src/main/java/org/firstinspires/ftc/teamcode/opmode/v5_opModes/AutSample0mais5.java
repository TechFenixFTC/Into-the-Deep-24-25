package org.firstinspires.ftc.teamcode.opmode.v5_opModes;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controller.Controladora;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Sugar.IntakeSuccao;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Sugar.SugarAngulationStates;
import org.firstinspires.ftc.teamcode.subsystems.agregadorasSubsistemas.Inferior.UnderGrounSubystemStates;
import org.firstinspires.ftc.teamcode.subsystems.agregadorasSubsistemas.Superior.UpperSubsystemStates;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.GarraOpeningStates;


@Config
@Autonomous(name = "Autonomo0+5", group = "Autonomous")
public class AutSample0mais5 extends LinearOpMode {
    V5 robot;
    Action push;
    public boolean encerrar = false;
    private double limeligtCorrection = 0;
    public static boolean girarAoContrario = false;
    Action proximaAction;

    public static double
            Xdeposit1 = -49 ,    Ydeposit1 = -57 ,  Hdeposit1 = 35,
            Xdeposit2 = -52 ,    Ydeposit2 = -53,  Hdeposit2 = 45,
            Xdeposit5 = -46 ,    Ydeposit5 = -62 ,  Hdeposit5 = 30,

            XCollect2 = -42 , YCollect2 = -42, HCollect2 = 89,
            LimelightXsample2 = 0.81,// distância entre o robo e a apriltag
            XCollect3 = -52 ,   YCollect3 = -42 , HCollect3 = 87,
            XCollect4 = -51 ,   YCollect4 = -39 , HCollect4 = 140,
            XCollect5 = -13 ,   YCollect5 = -5 , HCollect5 = 0, TangentCollect5 = 130, TangentCollect5part2 = 0,
            XCollect5part2 = -3,
            delaySoltarSample1 = 0.1, delaySoltarSample2 = 0.280, delaySoltarSample4 = 0.380, tangentCollect4 = 145, tangentDeposit4 = 145,
            LimelightDelayToRead = 0.400, currMax = 2.50
            ;





    public static double[] sample4 ={10,10,10};


    @Override
    public void runOpMode()  {

        robot = Controladora.create0mai5Robot(hardwareMap, telemetry);

        Actions.runBlocking(
                new SequentialAction(
                        robot.outtakeIntakeSuperior.garraSuperior.fecharGarra(),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                        robot.outtakeIntakeSuperior.braco.goToInital(),
                        robot.md.actionBuilder(robot.md.pose).waitSeconds(1).build(),
                        new InstantAction(() -> {robot.outtakeIntakeSuperior.garraSuperior.fecharGarra();}),
                        new InstantAction(() -> {robot.outtakeIntakeSuperior.garraSuperior.servoAngulacaoGarra.setPosition(0.625);}),
                        new InstantAction(() -> {robot.outtakeIntakeSuperior.garraSuperior.garraChamber();}),
                        new InstantAction(() -> {robot.intakeInferior.intakeSuccao.angulacao.setPosition(0.466);}),
                        //todo : mudar estados do robo para outtake
                        new InstantAction(() -> robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INITIAL),
                        new InstantAction(() -> robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.OUTTAKE)
                )
        );


        waitForStart();
        while (robot.controladora.sampleID <= 4) {

            encerrar = false;
            proximaAction = robot.controladora.decideProximaAcao(robot);
            if(proximaAction != null){
                Actions.runBlocking(
                        new ParallelAction(
                                //todo Roda a próxima ação retornada pela controladora em paralelo com nossas automatizações
                                AutomationSample(),
                                new SequentialAction(
                                        proximaAction,
                                        new InstantAction(() -> encerrar = true)
                                )

                        )
                );
            }

        }

    }


    public Action AutomationSample(){
        return new Action() {
            boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!started){
                    resetRuntime();
                    started =true;
                }

                robot.outtakeIntakeSuperior.monitorEstadosAutonomo(telemetryPacket);
                robot.intakeInferior.monitorEstadosAutonomo(telemetryPacket);
                robot.intakeInferior.intakeSuccao.colorSensorSugar.colorMatcher.monitorAutonomo(telemetryPacket);
                robot.runStatesSampleAutonomo(robot.carteiro, getRuntime(),robot);
                telemetryPacket.addLine("sampleID: "+ robot.controladora.sampleID);
                telemetryPacket.addLine("estadoAutonomo: "+ robot.controladora.estadoSample);
                telemetryPacket.addLine("quantas vezes mandou a próxima ação: " + robot.controladora.quantasVezesFoiExecutado);
                if(encerrar){
                    return false;
                }
                return true;

            }
        };
    }


    public Action esperarPeloIntakeTerDescido(){
        // verificar se um action já terminou para passar o estado para outra
        // por enquanto só para intake
        return new Action() {
            boolean IntakeSugarJaEstaEmIntake = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                IntakeSugarJaEstaEmIntake = robot.intakeInferior.intakeSuccao.sugarAngulationStates == SugarAngulationStates.INTAKE; //intake ja desceu

                if(IntakeSugarJaEstaEmIntake){
                    return false;
                }
                return true ;
            }
        };
    }


    public Action esperarPeloOutakeTerSubido(double delay){
        return new Action() {
            boolean jaSubiu = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetryPacket.addLine("Esperando pra extender");

                jaSubiu = robot.outtakeIntakeSuperior.linearVertical.motorR.getCurrentPosition() > 1200;

                if(jaSubiu){
                    telemetryPacket.addLine("Extendendo");
                    robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE;
                    return false;
                }



                return true;
            }
        };
    }
    public Action goToDeposit(Pose2d pose2d){//todo okey
        return robot.md.actionBuilder(pose2d)
                // basket
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(Xdeposit1, Ydeposit1, Math.toRadians(Hdeposit1)), Math.toRadians(-135), null, new ProfileAccelConstraint(-30, 30))
                //.waitSeconds(0.1)
                //.splineToLinearHeading(new Pose2d(Xdeposit1, Ydeposit1, Math.toRadians(Hdeposit1)), Math.toRadians(-135))
                .build();

    }
    public Action goToDeposit2(Pose2d pose2d){//todo okey
        return robot.md.actionBuilder(pose2d)
                // basket
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(Xdeposit2, Ydeposit2, Math.toRadians(Hdeposit2)), Math.toRadians(-135))
                .build();

    }
    public Action goToDeposit5(Pose2d pose2d){//todo okey
        return robot.md.actionBuilder(pose2d)
                // basket
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(Xdeposit5, Ydeposit5, Math.toRadians(Hdeposit5)), Math.toRadians(-135))
                .build();

    }

    public Action testIIntake(){
        return new InstantAction(()->{
            robot.carteiro.addOrder( new InstantAction(() -> robot.intakeInferior.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE), 0, "inferior pra intake", getRuntime());
            robot.carteiro.addOrder(new InstantAction(()->  robot.outtakeIntakeSuperior.upperSubsystemStates = UpperSubsystemStates.INITIAL), 0, "superior pra initial", getRuntime());
        });
    }
    public Action GoCollectSample2() {//todo okey

        Action move1 = robot.md.actionBuilder(new Pose2d(Xdeposit1, Ydeposit1, Math.toRadians(Hdeposit1)))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(XCollect2, YCollect2 - 9,Math.toRadians(HCollect2 - 10)), Math.toRadians(90))
                .build();

        return move1;


    }

    public Action GoCollectPartido() {//todo okey
        return robot.md.actionBuilder(new Pose2d(Xdeposit1, Ydeposit1, Hdeposit1))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(XCollect2, YCollect2 - 8,Math.toRadians(HCollect2)), Math.toRadians(90))
                .build();
    }

    public Action LimelightCalculateCorrection(double targetX) {
        return new Action() {


            final ElapsedTime temporizador = new ElapsedTime();
            boolean started = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                /* todo: iniciamos o cronômetro assim que o código inicia */
                if(!started){
                    temporizador.reset();
                }


                /*todo: Loop da ação*/







                /*todo: depois que um tempo prédeterminado passa, a action vai terminar, com o último resultado calculado sendo salvo pra podermos usar em outros pontos*/
                if(temporizador.time() >= LimelightDelayToRead){
                    limeligtCorrection = 0;
                    return  false;
                }
                return true;
            }
        };
    }

    public Action CollectS2part2() {



        Action move2 =
                 robot.md.actionBuilder(new Pose2d(XCollect2, YCollect2 - 9,Math.toRadians(HCollect2 - 10)))
                .splineToSplineHeading(new Pose2d(XCollect2 + limeligtCorrection,YCollect2, Math.toRadians(HCollect2)), Math.toRadians(90))
                .build();

        return move2;
    }
    public Action GoCollectSample3() {//todo okey
        return robot.md.actionBuilder(new Pose2d(Xdeposit2, Ydeposit2, Math.toRadians(Hdeposit2)))

                // collect Sample 3
                .setTangent(Math.toRadians(135))

                .splineToLinearHeading(new Pose2d(XCollect3, YCollect3 - 8, Math.toRadians(HCollect3)), Math.toRadians(90))
                .waitSeconds(0.4)
                .splineToSplineHeading(new Pose2d(XCollect3, YCollect3, Math.toRadians(HCollect3)), Math.toRadians(90))
                .build();

    }
    public Action GoCollectSample4() {//todo okey
        return robot.md.actionBuilder(new Pose2d(Xdeposit2, Ydeposit2, Math.toRadians(Hdeposit2)))

                // collect Sample 4
                .setTangent(Math.toRadians(HCollect4))
                .splineToLinearHeading(new Pose2d(XCollect4 + 10, YCollect4 - 10, Math.toRadians(HCollect4)), Math.toRadians(tangentCollect4))
                .waitSeconds(0.4)
                .splineToSplineHeading(new Pose2d(XCollect4, YCollect4, Math.toRadians(HCollect4)), Math.toRadians(tangentCollect4))
                .build();
    }

    public Action goToDeposit4(Pose2d pose2d){//todo okey
        return robot.md.actionBuilder(pose2d)
                // basket
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(Xdeposit2, Ydeposit2, Math.toRadians(Hdeposit2)), Math.toRadians(tangentDeposit4))
                .build();

    }

    public Action GoCollectSample5() {//todo okey
        return new SequentialAction(
          new InstantAction(() -> {
              IntakeSuccao.power_Sugador = 0.7;
          }),
                robot.md.actionBuilder(new Pose2d(Xdeposit2, Ydeposit2, Math.toRadians(Hdeposit2)))

                        // collect Sample 4
                        .setTangent(Math.toRadians(TangentCollect5))
                        .splineToLinearHeading(new Pose2d(XCollect5, YCollect5, Math.toRadians(HCollect5)), Math.toRadians(TangentCollect5part2))
                        //.strafeToLinearHeading(new Vector2d(XCollect5, YCollect5), Math.toRadians(HCollect5))
                        .build()
        );
    }
    public Action GoCollectSample5Part2() {//todo okey
        return robot.md.actionBuilder(new Pose2d(XCollect5, YCollect5, Math.toRadians(HCollect5)))

                // collect Sample 4
                .setTangent(Math.toRadians(TangentCollect5part2))
                .splineToLinearHeading(new Pose2d(XCollect5part2, YCollect5, Math.toRadians(HCollect5)), Math.toRadians(0))
                .build();
    }
}