package org.firstinspires.ftc.teamcode.subsystems.agregadorasSubsistemas.Inferior;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5Modes;
import org.firstinspires.ftc.teamcode.opmode.v5_opModes.AutSample0mais5;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Controller.OrdersManager;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Garra.GarraInferior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Horizontal.LinearHorizontalMotor;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Sugar.AlcapaoStates;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Sugar.IntakeSuccao;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Sugar.SugarAngulationStates;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Horizontal.LinearHorizontalStates;
import org.firstinspires.ftc.teamcode.common.controls.MatchColor;

import java.security.PublicKey;

public class SubsistemasInferiores {
    public boolean teleopAllianceAzul = false;
    public MatchColor matchColor;
    double cooldown = 0, lastTimeB = 0, runtTime = 0;
    public Telemetry telemetry;
    public IntakeSuccao intakeSuccao;
    public UnderGrounSubystemStates underGrounSubystemStates = UnderGrounSubystemStates.INITIAL;
    public static boolean monitorEstadosInferiores = true, mandouRetrairEmAlgumPonto = false, mandouSubirOIntake = false,alcapaoManual =false;

    public GarraInferior garraInferior;
    boolean estaEjetando = false;
    public LinearHorizontalMotor linearHorizontalMotor;
    HardwareMap hardwaremap;
    public SubsistemasInferiores(HardwareMap hardwareMap, Telemetry telemetry) {
        this.matchColor = new MatchColor();
        this.hardwaremap = hardwareMap;
        this.telemetry = telemetry;
        this.linearHorizontalMotor = new LinearHorizontalMotor(hardwareMap);
        this.intakeSuccao = new IntakeSuccao(hardwareMap);
    }
  /*****************************************/
  /*********** Funções Antigas  ************/
  /*****************************************/
    private double getVoltage() { return hardwaremap.voltageSensor.iterator().next().getVoltage(); }
    public void ejectingSampleWrong(OrdersManager carteiro, double runtime, V5Modes v5Mode, String ladoAliança) {

    }
    public void goToInitial_goToReadyTransfer(OrdersManager carteiro, double runtime){
        underGrounSubystemStates = UnderGrounSubystemStates.INITIAL;
        carteiro.addOrder(intakeSuccao.TransferPositionAlcapao(), 0, "alcapao transfer", runtime);
        carteiro.addOrder(actionIntakeIrPraInitial(),0,"horizonte",runtime);
    }
    public Action actionIntakeIrPraInitial() {

        Action returnedAction;

        returnedAction = new SequentialAction(
                // muda o estado para going to retracted
                actionLevantarAntesDeRetrair(),
                linearHorizontalMotor.goToRetracted(),
                intakeSuccao.GoToTransfer()
        );

        return returnedAction;
    }
    public Action actionLevantarAntesDeRetrair() {
        if (linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.RETRACTED){ // se o horizontal ja estiver retraido não rola nada
            return new InstantAction(() -> {});
        }
        else { // se não ele levanta o intake antes de retrair
            return new SequentialAction(
                    intakeSuccao.GotoReadyToIntakeSample(),
                    new MecanumDrive(hardwaremap, new Pose2d(0,0,0)).actionBuilder(new Pose2d(0,0,0)).waitSeconds(0.1).build()
            );
        }
    }
    public void goToTransfer(OrdersManager carteiro, double runtime)    {
        carteiro.addOrder(goToTransferAction(),0,"garra inferior", runtime);
    }
    public Action goToTransferAction()    {
        return new SequentialAction(
                linearHorizontalMotor.goToRetracted(),
                intakeSuccao.GoToTransfer()
        );
    }
    public void goToReadyToIntake(OrdersManager carteiro, double runtime){
        this.underGrounSubystemStates = UnderGrounSubystemStates.READY_TOINTAKE;
        carteiro.addOrder(intakeSuccao.ReadytoIntakePositionAlcapao(),0.0,"alcapao aberto3",runtime);
        carteiro.addOrder(intakeSuccao.GotoReadyToIntakeSpecimen(),0.0,"angulation",runtime);
        carteiro.addOrder(linearHorizontalMotor.goToExtended(),0.4,"horizonte",runtime);
    }
    public void goToReadyToIntakeSample(OrdersManager carteiro, double runtime){
        this.underGrounSubystemStates = UnderGrounSubystemStates.READY_TOINTAKE;
        carteiro.addOrder(intakeSuccao.ReadytoIntakePositionAlcapao(),0.0,"alcapao aberto4",runtime);
        carteiro.addOrder(intakeSuccao.GotoReadyToIntakeSpecimen(),0.0,"angulation",runtime);
        carteiro.addOrder(linearHorizontalMotor.goToExtended(),0.4,"horizonte",runtime);
    }
    public void goToIntakeSample(OrdersManager carteiro, double runtime){
        underGrounSubystemStates = UnderGrounSubystemStates.INTAKE;
        carteiro.addOrder(linearHorizontalMotor.goToExtended(),0,"horizonte",runtime);
        carteiro.addOrder(intakeSuccao.IntakeSamplePositionAlcapao(),0.2,"alcapao aberto2",runtime);
        carteiro.addOrder(intakeSuccao.GotoIntakeSample(),0,"angulation",runtime);
    }
    public void goToIntakeSpecimen(OrdersManager carteiro, double runtime){
        this.underGrounSubystemStates = UnderGrounSubystemStates.INTAKE;
        carteiro.addOrder(intakeSuccao.TransferPositionAlcapao(), 0, "alcapao intake", runtime);
        carteiro.addOrder(linearHorizontalMotor.goToExtended(),0,"horizonte",runtime);
        carteiro.addOrder(intakeSuccao.GotoIntakeSpecimen(),0,"angulation",runtime);
    }

  /*****************************************/
  /******** Funções modo Specimen **********/
  /*****************************************/
  //todo teleop
    public void runStatesSpecimen(OrdersManager carteiro, double runtime, V5 robot, GamepadEx gamepad) {
        if(underGrounSubystemStates == UnderGrounSubystemStates.INITIAL) {
            InitiaSpecimen(carteiro, runtime, robot, gamepad);
        }
        if(underGrounSubystemStates == UnderGrounSubystemStates.INTAKE) {

        }


  }
    public  void  InitiaSpecimen(OrdersManager carteiro, double runtime, V5 robot, GamepadEx gamepad){
        boolean linearEstaRetraido         = linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.RETRACTED;
        boolean intakeSuccaoTaNoModoIntake = intakeSuccao.sugarAngulationStates == SugarAngulationStates.INTAKE;
        boolean alcapaoPosicaoProTransfer  = intakeSuccao.alcapaoStates == AlcapaoStates.TRASNFER;
        boolean alcapaoPosicaoPraIntake    = intakeSuccao.alcapaoStates == AlcapaoStates.INTAKE_SAMPLE;
        boolean temUmaSampleNoIntake       = intakeSuccao.colorSensorSugar.colorMatcher.temUmaSampleNoIntake();
        boolean samplePosicaoPraTransfer   = intakeSuccao.colorSensorSugar.colorMatcher.sampleNaPosicaoCorreta();
        boolean usandoControleManual       = gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0 || gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0;
        double delay = 0;
        // AÇÕES DE TRANSIÇÃO Automática
        /* todo: ir para Transfer quando> 1- tiver uma sample na posição 2- alçapão pra transfer 3- horizontal retraido*/
        if(samplePosicaoPraTransfer && alcapaoPosicaoProTransfer && linearEstaRetraido) {
            carteiro.addOrder(new InstantAction(() -> underGrounSubystemStates = UnderGrounSubystemStates.TRANSFER), 0, "Mudar modo subsistemas inferiores pra transfer", runtime);
        }
        // AÇÕES DO ESTADO
        /* todo: levanto o intake antes de retrair */
        if(!linearEstaRetraido && intakeSuccaoTaNoModoIntake && !linearHorizontalMotor.isBusy) {
            delay = 0.1;
            if (!carteiro.hasOrder("Intake Sucção Pra ReadyToIntake")) {
                carteiro.addOrder(
                        intakeSuccao.GotoReadyToIntakeSample(),
                        0.07,
                        "Intake Sucção Pra ReadyToIntake",
                        runtime
                );
            }

        }

        /* todo: retrai o horizontal*/
        if(!linearEstaRetraido) {
            carteiro.addOrder(linearHorizontalMotor.goToRetracted(), delay, "retrairLinear", runtime);
        }
        /* todo: levanta o alçapão*/
        if(!alcapaoPosicaoProTransfer){
            carteiro.addOrder(intakeSuccao.IntakeSamplePositionAlcapao(), 0, "alcapao transfer", runtime);
        }

    }



  /*****************************************/
  /******** Funções modo Sample ************/
  /*****************************************/
  //todo teleop
    public void runStatesSample(OrdersManager carteiro, double runtime, V5 robot, GamepadEx gamepad) {

        if(underGrounSubystemStates == UnderGrounSubystemStates.INITIAL || underGrounSubystemStates == UnderGrounSubystemStates.TRANSFER ) {
            Initial_ReadyToTransferSample(carteiro, runtime, robot, gamepad);
        }
        if(underGrounSubystemStates == UnderGrounSubystemStates.INTAKE) {
            IntakeSample2(carteiro, runtime, robot, gamepad);
        }

    }
    public void Initial_ReadyToTransferSample(OrdersManager carteiro, double runtime, V5 robot, GamepadEx gamepad) {
        boolean linearEstaRetraido         = linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.RETRACTED && linearHorizontalMotor.sensorIsDetectingHorintontal();
        boolean intakeSuccaoTaNoModoIntake = intakeSuccao.sugarAngulationStates == SugarAngulationStates.INTAKE;
        boolean alcapaoPosicaoProTransfer  = intakeSuccao.alcapaoStates == AlcapaoStates.TRASNFER;
        boolean alcapaoPosicaoPraIntake    = intakeSuccao.alcapaoStates == AlcapaoStates.INTAKE_SAMPLE;
        boolean temUmaSampleNoIntake       = intakeSuccao.colorSensorSugar.colorMatcher.temUmaSampleNoIntake();
        boolean samplePosicaoPraTransfer   = intakeSuccao.colorSensorSugar.colorMatcher.sampleNaPosicaoCorreta();
        boolean usandoInputSugador       = gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1 || gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1;
        double delay = 0;
        // AÇÕES DE TRANSIÇÃO Automática




        /* todo: ir para Transfer quando> 1- tiver uma sample na posição 2- alçapão pra transfer 3- horizontal retraido*/
        if(samplePosicaoPraTransfer && alcapaoPosicaoProTransfer && linearEstaRetraido && underGrounSubystemStates != UnderGrounSubystemStates.TRANSFER) {

            carteiro.addOrder(new InstantAction(() -> underGrounSubystemStates = UnderGrounSubystemStates.TRANSFER), 0, "Mudar modo subsistemas inferiores pra transfer", runtime);

        }else if( underGrounSubystemStates != UnderGrounSubystemStates.INITIAL && !temUmaSampleNoIntake) {
            carteiro.addOrder(new InstantAction(() -> underGrounSubystemStates = UnderGrounSubystemStates.INITIAL), 0, "Mudar modo subsistemas inferiores pra inicial", runtime);
        }


        // AÇÕES DO ESTADO
        /* todo: levanto o intake antes de retrair */
        if(!linearEstaRetraido && intakeSuccaoTaNoModoIntake && !linearHorizontalMotor.isBusy) {
            delay = 0.1;
            if (!carteiro.hasOrder("Intake Sucção Pra ReadyToIntake")) {
                carteiro.addOrder(
                        intakeSuccao.GotoReadyToIntakeSample(),
                        0.07,
                        "Intake Sucção Pra ReadyToIntake",
                        runtime
                );
            }

        }
        else {
            intakeSuccao.GoToTransfer();
        }
        /* todo: retrai o horizontal*/
        linearHorizontalMotor.PID();
        if(!linearEstaRetraido && !linearHorizontalMotor.isBusy) {
            if(!carteiro.hasOrder("retrairLinear")){
                carteiro.addOrder(linearHorizontalMotor.goToRetracted(), delay, "retrairLinear", runtime);
            }

        }

        /* todo: Manter pressionando o horiontal enquanto tiver uma sample no intake e o horizontal estiver retraido */
        if(linearEstaRetraido) {
            if(temUmaSampleNoIntake && LinearHorizontalMotor.targetPosition != -10){
                LinearHorizontalMotor.targetPosition = -10;
            }else if(!temUmaSampleNoIntake && LinearHorizontalMotor.targetPosition != 25) {
                LinearHorizontalMotor.targetPosition = 25;
            }
        }

        /* todo: Ações do intake de sucção quando tem uma sampleNoIntake */
        if(temUmaSampleNoIntake){
            if(samplePosicaoPraTransfer){
                /* todo: alçapão na posição de transfer */
                if(linearEstaRetraido && !alcapaoPosicaoProTransfer) {
                    carteiro.addOrder(intakeSuccao.TransferPositionAlcapao(), 0, "alcapao transfer", runtime);
                }
            }
        }

        /* todo: Ações do intake de sucção quando NÃO tem uma sampleNoIntake */
        //todo se o alcapão

        if(!samplePosicaoPraTransfer) {
            /* todo: alçapão pra posição onde a sample consiga entrar e encaiaxr */
            if(linearEstaRetraido && !alcapaoPosicaoPraIntake) {
                carteiro.addOrder(intakeSuccao.IntakeSamplePositionAlcapao(), 0, "alcapao transfer", runtime);
            }
            /* todo: enquanto a sample não entra, ficar sugando ali */
            if(!usandoInputSugador) {
                if(temUmaSampleNoIntake) {
                    carteiro.addOrder( intakeSuccao.IntakeAjeitarSample(), 0, "sugador", runtime);
                }
                if(!temUmaSampleNoIntake) {
                    carteiro.addOrder( intakeSuccao.IntakeParar(), 0, "sugador", runtime);
                }


            }else {
                intakeSuccao.gerenciadorDoSugadorManual(gamepad, runtime);

            }
        }else {
            intakeSuccao.gerenciadorDoSugadorManual(gamepad, runtime);
        }
    }
    public void IntakeSample(OrdersManager carteiro, double runtime, V5 robot, GamepadEx gamepad){
        boolean linearEstaRetraido               = linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.RETRACTED;
        boolean linearEstaEstendido              = linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.EXTENDED;
        boolean intakeSuccaoTaNoModoIntake       = intakeSuccao.sugarAngulationStates == SugarAngulationStates.INTAKE;
        boolean intakeSuccaoTaNoModoInitial      = intakeSuccao.sugarAngulationStates == SugarAngulationStates.INITIAL;
        boolean intakeSuccaoTaNoModoReadyIntake  = intakeSuccao.sugarAngulationStates == SugarAngulationStates.READY_TOINTAKE;

        boolean alcapaoPosicaoProTransfer        = intakeSuccao.alcapaoStates == AlcapaoStates.TRASNFER;
        boolean alcapaoPosicaoPraIntakeSample    = intakeSuccao.alcapaoStates == AlcapaoStates.INTAKE_SAMPLE;
        boolean temUmaSampleNoIntake             = intakeSuccao.colorSensorSugar.colorMatcher.temUmaSampleNoIntake();
        boolean samplePosicaoPraTransfer         = intakeSuccao.colorSensorSugar.colorMatcher.sampleNaPosicaoCorreta();
        boolean usandoInputAngulacao             = gamepad.getButton(GamepadKeys.Button.X);
        boolean usandoInputSugador               = (gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1 || gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1);
        double delay = 0;

        linearHorizontalMotor.PID();
        /*todo: estica o horizontal*/
        if(linearEstaRetraido){
            carteiro.addOrder(linearHorizontalMotor.goToExtended(), delay, "ExtenderLinear", runtime);
            if (!carteiro.hasOrder("Intake Sucção Pra ReadyToIntake")) {
                carteiro.addOrder(
                        intakeSuccao.GotoReadyToIntakeSample(),
                        0.17,
                        "Intake Sucção Pra ReadyToIntake",
                        runtime
                );
            }

        }
        /* todo: alçapão para a posição de intake sample(abaixado) */
        if(!alcapaoPosicaoPraIntakeSample){
            carteiro.addOrder(intakeSuccao.IntakeSamplePositionAlcapao(), 0, "alcapao intake sample", runtime);
        }
        /*todo: Angulação do intake*/
        if(robot.teelop) {
            if(!linearEstaEstendido) {
                intakeSuccao.GotoReadyToIntakeSample();
            }
            else if(usandoInputAngulacao) {
                gerenciadorIntakeSample(carteiro, runtime);
            }
        }

        /*todo: Sugador*/
        if(usandoInputSugador){
            intakeSuccao.gerenciadorDoSugadorManual(gamepad, runtime);
        }
        if(intakeSuccaoTaNoModoIntake || samplePosicaoPraTransfer) {
            if(!usandoInputSugador) {
                carteiro.addOrder( intakeSuccao.IntakeSugar(), 0, "sugador", runtime);

            }
        }else {
            if(!usandoInputSugador) {
                carteiro.addOrder( intakeSuccao.IntakeParar(), 0, "sugador", runtime);

            }
        }
        /*todo: Sugou certo, retraiu*/
        if(samplePosicaoPraTransfer && linearEstaEstendido && !linearHorizontalMotor.isBusy) {
            carteiro.addOrder(new InstantAction(() -> underGrounSubystemStates = UnderGrounSubystemStates.INITIAL), 0, "ir pra modo Inicial", runtime);
        }


    }
    public void IntakeSample2(OrdersManager carteiro, double runtime, V5 robot, GamepadEx gamepad){
        boolean linearEstaRetraido               = linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.RETRACTED;
        boolean SugouSampleErrado                = false;
        boolean linearEstaEstendido              = linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.EXTENDED;
        boolean intakeSuccaoTaNoModoIntake       = intakeSuccao.sugarAngulationStates == SugarAngulationStates.INTAKE;
        boolean intakeSuccaoTaNoModoInitial      = intakeSuccao.sugarAngulationStates == SugarAngulationStates.INITIAL;
        boolean intakeSuccaoTaNoModoReadyIntake  = intakeSuccao.sugarAngulationStates == SugarAngulationStates.READY_TOINTAKE;


        boolean usandoInputAlcapao               = gamepad.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON);
        boolean alcapaoPosicaoProTransfer        = intakeSuccao.alcapaoStates == AlcapaoStates.TRASNFER;
        boolean alcapaoPosicaoPraIntakeSample    = intakeSuccao.alcapaoStates == AlcapaoStates.INTAKE_SAMPLE;
        boolean temUmaSampleNoIntake             = intakeSuccao.colorSensorSugar.colorMatcher.temUmaSampleNoIntake();
        boolean samplePosicaoPraTransfer         = intakeSuccao.colorSensorSugar.colorMatcher.sampleNaPosicaoCorreta();
        boolean usandoInputAngulacao             = gamepad.getButton(GamepadKeys.Button.X);
        boolean usandoInputSugador               = (gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1 || gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1);
        double delay = 0;

        /*todo: ver qual cor vai descartar*/
        if(teleopAllianceAzul){
            SugouSampleErrado = intakeSuccao.colorSensorSugar.colorMatcher.getSampleColor().equals("Vermelho");
        }
        else {
            SugouSampleErrado = intakeSuccao.colorSensorSugar.colorMatcher.getSampleColor().equals("Azul");
        }

        linearHorizontalMotor.PID();
        /*todo: estica o horizontal*/
        if(linearEstaRetraido){
            carteiro.addOrder(linearHorizontalMotor.goToExtended(), delay, "ExtenderLinear", runtime);
            if (!carteiro.hasOrder("Intake Sucção Pra ReadyToIntake")) {
                carteiro.addOrder(
                        intakeSuccao.GotoReadyToIntakeSample(),
                        0.17,
                        "Intake Sucção Pra ReadyToIntake",
                        runtime
                );
            }

        }
        /* todo: alçapão para a posição de intake sample(abaixado) */

        if(usandoInputAlcapao){
            intakeSuccao.gerenciadorDoAlcapaoManual(carteiro,runtime);
            alcapaoManual  = true;
        }
        if(!alcapaoPosicaoPraIntakeSample && !alcapaoManual){
            carteiro.addOrder(intakeSuccao.IntakeSamplePositionAlcapao(), 0, "alcapao intake sample", runtime);
        }
        /*todo: Angulação do intake*/
        if(robot.teelop) {
            if(!linearEstaEstendido) {
                intakeSuccao.GotoReadyToIntakeSample();
            }
            else if(usandoInputAngulacao) {
                gerenciadorIntakeSample(carteiro, runtime);
            }
        }

        /*todo: Sugador*/
        if(usandoInputSugador){
            intakeSuccao.gerenciadorDoSugadorManual(gamepad, runtime);
        }
        if(intakeSuccaoTaNoModoIntake || samplePosicaoPraTransfer) {
            if(!usandoInputSugador) {
                carteiro.addOrder( intakeSuccao.IntakeSugar(), 0, "sugador", runtime);

            }
        }else {
            if(!usandoInputSugador) {
                carteiro.addOrder( intakeSuccao.IntakeParar(), 0, "sugador", runtime);

            }
        }
        /*todo: sugou sample errado */
        if(temUmaSampleNoIntake) {
            carteiro.addOrder(intakeSuccao.GotoReadyToIntakeSpecimen(), 0, "ir pra modo ready", runtime);


            if(SugouSampleErrado) {
                if(!carteiro.hasOrder("ejetar")){
                    estaEjetando = true;
                    carteiro.addOrder(intakeSuccao.TotalOpenPositionAlcapao(), 0,"ejetar", runtime);

                    if(!carteiro.hasOrder("nao esta ejetando")){
                        carteiro.addOrder(new InstantAction(() -> estaEjetando = false), 0.3, "nao esta ejetando", runtime);
                        carteiro.addOrder(new InstantAction(() -> underGrounSubystemStates = UnderGrounSubystemStates.INTAKE), 0.4, "ir para modo Intake", runtime);
                    }
                }
            }
            if(samplePosicaoPraTransfer && linearEstaEstendido && !linearHorizontalMotor.isBusy && !SugouSampleErrado) {
                carteiro.addOrder(new InstantAction(() -> underGrounSubystemStates = UnderGrounSubystemStates.INITIAL), 0, "ir pra modo Inicial", runtime);
            }
        }

    }

  //todo auto
    public void runStatesSampleAutonomo(OrdersManager carteiro, double runtime, V5 robot) {

        if(underGrounSubystemStates == UnderGrounSubystemStates.INITIAL || underGrounSubystemStates == UnderGrounSubystemStates.TRANSFER) {
            Initial_ReadyToTransferSampleAutonomo(carteiro, runtime, robot);
        }
        if(underGrounSubystemStates == UnderGrounSubystemStates.INTAKE) {
            IntakeSampleAutonomo(carteiro, runtime, robot);
        }
        if(underGrounSubystemStates == UnderGrounSubystemStates.READY_TOINTAKE) {
            ReadyToIntakeSampleAutonomo(carteiro, runtime, robot);
        }


    }
    public void Initial_ReadyToTransferSampleAutonomo(OrdersManager carteiro, double runtime, V5 robot) {
        boolean linearEstaRetraido         = linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.RETRACTED && linearHorizontalMotor.sensorIsDetectingHorintontal();
        boolean intakeSuccaoTaNoModoIntake = intakeSuccao.sugarAngulationStates == SugarAngulationStates.INTAKE;
        boolean alcapaoPosicaoProTransfer  = intakeSuccao.alcapaoStates == AlcapaoStates.TRASNFER;
        boolean alcapaoPosicaoPraIntake    = intakeSuccao.alcapaoStates == AlcapaoStates.INTAKE_SAMPLE;
        boolean temUmaSampleNoIntake       = intakeSuccao.colorSensorSugar.colorMatcher.temUmaSampleNoIntake();
        boolean samplePosicaoPraTransfer   = intakeSuccao.colorSensorSugar.colorMatcher.sampleNaPosicaoCorreta();
        double delay = 0;

        this.runtTime = runtime;
        // AÇÕES DE TRANSIÇÃO Automática
        /* todo: ir para Transfer quando> 1- tiver uma sample na posição 2- alçapão pra transfer 3- horizontal retraido*/
        if(samplePosicaoPraTransfer && alcapaoPosicaoProTransfer && linearEstaRetraido && underGrounSubystemStates != UnderGrounSubystemStates.TRANSFER) {
            mandouSubirOIntake = false;
            mandouRetrairEmAlgumPonto = false;
            carteiro.addOrder(new InstantAction(() -> underGrounSubystemStates = UnderGrounSubystemStates.TRANSFER), 0, "Mudar modo subsistemas inferiores pra transfer", runtime);


        }else if( underGrounSubystemStates != UnderGrounSubystemStates.INITIAL && !temUmaSampleNoIntake) {
            carteiro.addOrder(new InstantAction(() -> underGrounSubystemStates = UnderGrounSubystemStates.INITIAL), 0, "Mudar modo subsistemas inferiores pra initial", runtime);
            //underGrounSubystemStates = UnderGrounSubystemStates.INITIAL;
        }


        // AÇÕES DO ESTADO
        /* todo: levanto o intake antes de retrair */
        if(!linearEstaRetraido && intakeSuccaoTaNoModoIntake && !linearHorizontalMotor.isBusy) {
            delay = 0.1;
            if (!carteiro.hasOrder("Intake Sucção Pra ReadyToIntake")) {
                mandouSubirOIntake = true;
                carteiro.addOrder(
                        intakeSuccao.GotoReadyToIntakeSample(),
                        0.07,
                        "Intake Sucção Pra ReadyToIntake",
                        runtime
                );
            }

        }
        else {
            intakeSuccao.GoToTransfer();
        }
        /* todo: retrai o horizontal*/
        linearHorizontalMotor.PID();
        if(!linearEstaRetraido && !linearHorizontalMotor.isBusy) {
            if(!carteiro.hasOrder("retrairLinear")){
                mandouRetrairEmAlgumPonto = true;
                carteiro.addOrder(linearHorizontalMotor.goToRetracted(), delay, "retrairLinear", runtime);
            }

        }

        /* todo: Manter pressionando o horiontal enquanto tiver uma sample no intake e o horizontal estiver retraido */
        if(linearEstaRetraido) {
            if(temUmaSampleNoIntake && LinearHorizontalMotor.targetPosition != -10){
                LinearHorizontalMotor.targetPosition = -10;
            }else if(!temUmaSampleNoIntake && LinearHorizontalMotor.targetPosition != 25) {
                LinearHorizontalMotor.targetPosition = 25;
            }
        }

        /* todo: Ações do intake de sucção quando tem uma sampleNoIntake */
        if(temUmaSampleNoIntake){
            if(samplePosicaoPraTransfer){
                /* todo: alçapão na posição de transfer */
                if(linearEstaRetraido && !alcapaoPosicaoProTransfer) {
                    carteiro.addOrder(intakeSuccao.TransferPositionAlcapao(), 0, "alcapao transfer", runtime);
                }
            }
        }

        /* todo: Ações do intake de sucção quando NÃO tem uma sampleNoIntake */
        if(!samplePosicaoPraTransfer) {
            /* todo: alçapão pra posição onde a sample consiga entrar e encaiaxr */
            if(linearEstaRetraido && !alcapaoPosicaoPraIntake) {
                carteiro.addOrder(intakeSuccao.IntakeSamplePositionAlcapao(), 0, "alcapao transfer", runtime);
            }
            /* todo: enquanto a sample não entra, ficar sugando ali */
                if(temUmaSampleNoIntake) {
                    if(AutSample0mais5.girarAoContrario){
                        carteiro.addOrder( intakeSuccao.IntakeRepelir(), 0, "sugador", runtime);
                        if(!carteiro.hasOrder("desligar")){
                            carteiro.addOrder(new InstantAction(() -> AutSample0mais5.girarAoContrario = false), 0.05, "desligar", runtime);
                        }
                    }else{
                        carteiro.addOrder( intakeSuccao.IntakeAjeitarSample(), 0, "sugador", runtime);
                    }

                }
                if(!temUmaSampleNoIntake) {
                    carteiro.addOrder( intakeSuccao.IntakeParar(), 0, "sugador", runtime);
                }



        }





    }
    public void ReadyToIntakeSampleAutonomo(OrdersManager carteiro, double runtime, V5 robot){
        int posicaoQueDaPraAbaixarOIntake = 1000;
        boolean linearTaNaPosicaoQueDaPraAbaixarOIntake = linearHorizontalMotor.motorHorizontal.getCurrentPosition() >= posicaoQueDaPraAbaixarOIntake;
        boolean linearEstaExtendendo             = LinearHorizontalMotor.targetPosition > 1200;
        boolean subsistemasEstaInicial           = underGrounSubystemStates == UnderGrounSubystemStates.INITIAL;
        boolean linearEstaRetraido               = linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.RETRACTED;
        boolean linearEstaEstendido              = linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.EXTENDED;
        boolean intakeSuccaoTaNoModoIntake       = intakeSuccao.sugarAngulationStates == SugarAngulationStates.INTAKE;
        boolean intakeSuccaoTaNoModoInitial      = intakeSuccao.sugarAngulationStates == SugarAngulationStates.INITIAL;
        boolean intakeSuccaoTaNoModoReadyIntake  = intakeSuccao.sugarAngulationStates == SugarAngulationStates.READY_TOINTAKE;

        boolean alcapaoPosicaoProTransfer        = intakeSuccao.alcapaoStates == AlcapaoStates.TRASNFER;
        boolean alcapaoPosicaoPraIntakeSample    = intakeSuccao.alcapaoStates == AlcapaoStates.INTAKE_SAMPLE;
        boolean temUmaSampleNoIntake             = intakeSuccao.colorSensorSugar.colorMatcher.temUmaSampleNoIntake();
        boolean samplePosicaoPraTransfer         = intakeSuccao.colorSensorSugar.colorMatcher.sampleNaPosicaoCorreta();
        double delay = 0;

        linearHorizontalMotor.PID();

        /* todo: ir para o estado inicial se tiver uma sample*/

        /*todo: estica o horizontal*/
        if(linearEstaRetraido){
            if(!carteiro.hasOrder("ExtenderLinear")){
                carteiro.addOrder(linearHorizontalMotor.goToExtended(), delay, "ExtenderLinear", runtime);
                if (!carteiro.hasOrder("Intake Sucção Pra ReadyToIntake")) {
                    carteiro.addOrder(
                            intakeSuccao.GotoReadyToIntakeSample(),
                            0.17,
                            "Intake Sucção Pra ReadyToIntake",
                            runtime
                    );
                }
            }


        }
        /* todo: alçapão para a posição de intake sample(abaixado) */
        if(!alcapaoPosicaoPraIntakeSample){
            carteiro.addOrder(intakeSuccao.IntakeSamplePositionAlcapao(), 0, "alcapao intake sample", runtime);
        }
        /*todo: Angulação do intake*/
        if(/*linearEstaExtendendo && linearTaNaPosicaoQueDaPraAbaixarOIntake*/linearEstaEstendido) {
            carteiro.addOrder(intakeSuccao.GotoReadyToIntakeSample(), 0, "Intake Sample", runtime);
        }





    }
    public void IntakeSampleAutonomo(OrdersManager carteiro, double runtime, V5 robot){
        int posicaoQueDaPraAbaixarOIntake = 1000;
        boolean linearTaNaPosicaoQueDaPraAbaixarOIntake = linearHorizontalMotor.motorHorizontal.getCurrentPosition() >= posicaoQueDaPraAbaixarOIntake;
        boolean linearEstaExtendendo             = LinearHorizontalMotor.targetPosition > 1200;
        boolean subsistemasEstaInicial           = underGrounSubystemStates == UnderGrounSubystemStates.INITIAL;
        boolean linearEstaRetraido               = linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.RETRACTED;
        boolean linearEstaEstendido              = linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.EXTENDED;
        boolean intakeSuccaoTaNoModoIntake       = intakeSuccao.sugarAngulationStates == SugarAngulationStates.INTAKE;
        boolean intakeSuccaoTaNoModoInitial      = intakeSuccao.sugarAngulationStates == SugarAngulationStates.INITIAL;
        boolean intakeSuccaoTaNoModoReadyIntake  = intakeSuccao.sugarAngulationStates == SugarAngulationStates.READY_TOINTAKE;

        boolean alcapaoPosicaoProTransfer        = intakeSuccao.alcapaoStates == AlcapaoStates.TRASNFER;
        boolean alcapaoPosicaoPraIntakeSample    = intakeSuccao.alcapaoStates == AlcapaoStates.INTAKE_SAMPLE;
        boolean temUmaSampleNoIntake             = intakeSuccao.colorSensorSugar.colorMatcher.temUmaSampleNoIntake();
        boolean samplePosicaoPraTransfer         = intakeSuccao.colorSensorSugar.colorMatcher.sampleNaPosicaoCorreta();
        double delay = 0;

        linearHorizontalMotor.PID();

        /* todo: ir para o estado inicial se tiver uma sample*/

        /*todo: estica o horizontal*/
        if(linearEstaRetraido){
            if(!carteiro.hasOrder("ExtenderLinear")){
                carteiro.addOrder(linearHorizontalMotor.goToExtended(), delay, "ExtenderLinear", runtime);
                if (!carteiro.hasOrder("Intake Sucção Pra ReadyToIntake")) {
                    carteiro.addOrder(
                            intakeSuccao.GotoReadyToIntakeSample(),
                            0.17,
                            "Intake Sucção Pra ReadyToIntake",
                            runtime
                    );
                }
            }


        }
        /* todo: alçapão para a posição de intake sample(abaixado) */
        if(!alcapaoPosicaoPraIntakeSample){
            carteiro.addOrder(intakeSuccao.IntakeSamplePositionAlcapao(), 0, "alcapao intake sample", runtime);
        }
        /*todo: Angulação do intake*/
            if(/*linearEstaExtendendo && linearTaNaPosicaoQueDaPraAbaixarOIntake*/linearEstaEstendido) {
                carteiro.addOrder(intakeSuccao.GotoIntakeSample(), 0, "Intake Sample", runtime);
            }




        /*todo: Sugador*/
        if(intakeSuccaoTaNoModoIntake || samplePosicaoPraTransfer) {
            carteiro.addOrder( intakeSuccao.IntakeSugar(), 0, "sugador", runtime);


        }else {
            carteiro.addOrder( intakeSuccao.IntakeParar(), 0, "sugador", runtime);
        }
        /*todo: Sugou certo, retraiu*/
        if(samplePosicaoPraTransfer && linearEstaEstendido && !linearHorizontalMotor.isBusy) {
            if(!carteiro.hasOrder("ir pra modo Inicial")){
                carteiro.addOrder(new InstantAction(() -> underGrounSubystemStates = UnderGrounSubystemStates.INITIAL), 0, "ir pra modo Inicial", runtime);
            }

        }


    }
    public void monitorEstados(){
        int estadoGeral = 0;
        if(underGrounSubystemStates == UnderGrounSubystemStates.TRANSFER) estadoGeral = -1;
        if(underGrounSubystemStates == UnderGrounSubystemStates.INTAKE) estadoGeral = 1;

        int estadoHorizontal = 0;
        if(linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.EXTENDED) estadoHorizontal = 1;

        int estadoAngular = 0;
        if(intakeSuccao.sugarAngulationStates == SugarAngulationStates.INTAKE) estadoAngular = 1;

        int estadoAlcapo  = 0;
        if(intakeSuccao.alcapaoStates == AlcapaoStates.INTAKE_SAMPLE) estadoAlcapo = 1;

        telemetry.addData("🚂 Estado Geral Inferiores", "%s (%d)", underGrounSubystemStates, estadoGeral);
        telemetry.addData("🛤️ Estado Horizontal", "%s (%d)", linearHorizontalMotor.linearHorizontalInferiorState, estadoHorizontal);
        telemetry .addData("🚇 Estado Angulação Intake", "%s (%d)", intakeSuccao.sugarAngulationStates, estadoAngular);
        telemetry .addData("🚪 Estado Alcapao Intake", "%s (%d)", intakeSuccao.alcapaoStates, estadoAlcapo);
    }


    public void monitorEstadosAutonomo(TelemetryPacket telemetry){
        int estadoGeral = 0;
        if(underGrounSubystemStates == UnderGrounSubystemStates.TRANSFER) estadoGeral = -1;
        if(underGrounSubystemStates == UnderGrounSubystemStates.INTAKE) estadoGeral = 1;

        int estadoHorizontal = 0;
        if(linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.EXTENDED) estadoHorizontal = 1;

        int estadoAngular = 0;
        if(intakeSuccao.sugarAngulationStates == SugarAngulationStates.INTAKE) estadoAngular = 1;

        int estadoAlcapo  = 0;
        if(intakeSuccao.alcapaoStates == AlcapaoStates.INTAKE_SAMPLE) estadoAlcapo = 1;

        telemetry.addLine(String.format("🚂 Estado Geral Inferiores: %s (%d) | RUNTIME: %s", underGrounSubystemStates, estadoGeral, this.runtTime));
        telemetry.addLine(String.format("🛤️ Estado Horizontal: %s (%d)", linearHorizontalMotor.linearHorizontalInferiorState, estadoHorizontal));
        telemetry.addLine(String.format("🚇 Estado Angulação Intake: %s (%d)", intakeSuccao.sugarAngulationStates, estadoAngular));
        telemetry.addLine(String.format("🚪 Estado Alçapão Intake: %s (%d)", intakeSuccao.alcapaoStates, estadoAlcapo));
        telemetry.addLine(String.format("Horizontal IsBusy: %s | Esta retraido: %s", linearHorizontalMotor.isBusy, linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.RETRACTED && linearHorizontalMotor.sensorIsDetectingHorintontal()));
        telemetry.addLine(String.format("Ordens ja feitas: SUBIR O INTAKE: %s | RETRAIR O HORIZONTAL: %s", mandouSubirOIntake, mandouRetrairEmAlgumPonto));
    }

  /*****************************************/
  /********* Funções V5 E MEIO *************/
  /*****************************************/
    public void runStatesSampleV5eMEIO(OrdersManager carteiro, double runtime, V5 robot, GamepadEx gamepad) {

      if(underGrounSubystemStates == UnderGrounSubystemStates.INITIAL || underGrounSubystemStates == UnderGrounSubystemStates.TRANSFER) {
          Initial_ReadyToTransferSampleV5eMEIO(carteiro, runtime, robot, gamepad);
      }
      if(underGrounSubystemStates == UnderGrounSubystemStates.INTAKE) {
          IntakeSampleV5eMEIO(carteiro, runtime, robot, gamepad);
      }

  }
    public void Initial_ReadyToTransferSampleV5eMEIO(OrdersManager carteiro, double runtime, V5 robot, GamepadEx gamepad) {
        boolean linearEstaRetraido         = linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.RETRACTED && linearHorizontalMotor.sensorIsDetectingHorintontal();
        boolean intakeSuccaoTaNoModoIntake = intakeSuccao.sugarAngulationStates == SugarAngulationStates.INTAKE;
        boolean alcapaoPosicaoProTransfer  = intakeSuccao.alcapaoStates == AlcapaoStates.TRASNFER;
        boolean alcapaoPosicaoPraIntake    = intakeSuccao.alcapaoStates == AlcapaoStates.INTAKE_SAMPLE;
        boolean temUmaSampleNoIntake       = intakeSuccao.colorSensorSugar.colorMatcher.temUmaSampleNoIntake();
        boolean samplePosicaoPraTransfer   = intakeSuccao.colorSensorSugar.colorMatcher.sampleNaPosicaoCorreta();
        boolean usandoInputSugador       = gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1 || gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1;
        double delay = 0;
        // AÇÕES DE TRANSIÇÃO Automática
        /* todo: ir para Transfer quando> 1- tiver uma sample na posição 2- alçapão pra transfer 3- horizontal retraido*/
        /*if(samplePosicaoPraTransfer && alcapaoPosicaoProTransfer && linearEstaRetraido && underGrounSubystemStates != UnderGrounSubystemStates.TRANSFER) {

            carteiro.addOrder(new InstantAction(() -> underGrounSubystemStates = UnderGrounSubystemStates.TRANSFER), 0, "Mudar modo subsistemas inferiores pra transfer", runtime);

        }else if( underGrounSubystemStates != UnderGrounSubystemStates.INITIAL && !temUmaSampleNoIntake) {
            carteiro.addOrder(new InstantAction(() -> underGrounSubystemStates = UnderGrounSubystemStates.INITIAL), 0, "Mudar modo subsistemas inferiores pra transfer", runtime);

        }*/


        // AÇÕES DO ESTADO
        /* todo: levanto o intake antes de retrair */
        if(!linearEstaRetraido && intakeSuccaoTaNoModoIntake && !linearHorizontalMotor.isBusy) {
            delay = 0.1;
            if (!carteiro.hasOrder("Intake Sucção Pra ReadyToIntake")) {
                carteiro.addOrder(
                        intakeSuccao.GotoReadyToIntakeSample(),
                        0.07,
                        "Intake Sucção Pra ReadyToIntake",
                        runtime
                );
            }

        }
        else {
            intakeSuccao.GoToTransfer();
        }
        /* todo: retrai o horizontal*/
        linearHorizontalMotor.PID();
        if(!linearEstaRetraido && !linearHorizontalMotor.isBusy) {
            carteiro.addOrder(linearHorizontalMotor.goToRetracted(), delay, "retrairLinear", runtime);
        }

        /* todo: Manter pressionando o horiontal enquanto tiver uma sample no intake e o horizontal estiver retraido */
        if(linearEstaRetraido) {
            if(temUmaSampleNoIntake && LinearHorizontalMotor.targetPosition != -10){
                LinearHorizontalMotor.targetPosition = -10;
            }else if(!temUmaSampleNoIntake && LinearHorizontalMotor.targetPosition != 25) {
                LinearHorizontalMotor.targetPosition = 25;
            }
        }

        /* todo: Ações do intake de sucção quando tem uma sampleNoIntake */
        if(temUmaSampleNoIntake){
            if(samplePosicaoPraTransfer){
                /* todo: alçapão na posição de transfer */
                if(linearEstaRetraido && !alcapaoPosicaoProTransfer) {
                    carteiro.addOrder(intakeSuccao.TransferPositionAlcapao(), 0, "alcapao transfer", runtime);
                }
            }
        }

        /* todo: Ações do intake de sucção quando NÃO tem uma sampleNoIntake */
        //todo se o alcapão

        if(!samplePosicaoPraTransfer) {
            /* todo: alçapão pra posição onde a sample consiga entrar e encaiaxr */
            if(linearEstaRetraido && !alcapaoPosicaoPraIntake) {
                carteiro.addOrder(intakeSuccao.IntakeSamplePositionAlcapao(), 0, "alcapao transfer", runtime);
            }
            /* todo: enquanto a sample não entra, ficar sugando ali */
            if(!usandoInputSugador) {
                if(temUmaSampleNoIntake) {
                    carteiro.addOrder( intakeSuccao.IntakeAjeitarSample(), 0, "sugador", runtime);
                }
                if(!temUmaSampleNoIntake) {
                    carteiro.addOrder( intakeSuccao.IntakeParar(), 0, "sugador", runtime);
                }


            }else {
                intakeSuccao.gerenciadorDoSugadorManual(gamepad, runtime);

            }
        }else {
            intakeSuccao.gerenciadorDoSugadorManual(gamepad, runtime);
        }





    }
    public void IntakeSampleV5eMEIO(OrdersManager carteiro, double runtime, V5 robot, GamepadEx gamepad){
        boolean linearEstaRetraido               = linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.RETRACTED;
        boolean linearEstaEstendido              = linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.EXTENDED;
        boolean intakeSuccaoTaNoModoIntake       = intakeSuccao.sugarAngulationStates == SugarAngulationStates.INTAKE;
        boolean intakeSuccaoTaNoModoInitial      = intakeSuccao.sugarAngulationStates == SugarAngulationStates.INITIAL;
        boolean intakeSuccaoTaNoModoReadyIntake  = intakeSuccao.sugarAngulationStates == SugarAngulationStates.READY_TOINTAKE;

        boolean alcapaoPosicaoProTransfer        = intakeSuccao.alcapaoStates == AlcapaoStates.TRASNFER;
        boolean alcapaoPosicaoPraIntakeSample    = intakeSuccao.alcapaoStates == AlcapaoStates.INTAKE_SAMPLE;
        boolean temUmaSampleNoIntake             = intakeSuccao.colorSensorSugar.colorMatcher.temUmaSampleNoIntake();
        boolean samplePosicaoPraTransfer         = intakeSuccao.colorSensorSugar.colorMatcher.sampleNaPosicaoCorreta();
        boolean usandoInputAngulacao             = gamepad.getButton(GamepadKeys.Button.X);
        boolean usandoInputSugador               = (gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1 || gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1);
        double delay = 0;

        linearHorizontalMotor.PID();
        /*todo: estica o horizontal*/
        if(linearEstaRetraido){
            carteiro.addOrder(linearHorizontalMotor.goToExtended(), delay, "ExtenderLinear", runtime);
            if (!carteiro.hasOrder("Intake Sucção Pra ReadyToIntake")) {
                carteiro.addOrder(
                        intakeSuccao.GotoReadyToIntakeSample(),
                        0.17,
                        "Intake Sucção Pra ReadyToIntake",
                        runtime
                );
            }

        }
        /* todo: alçapão para a posição de intake sample(abaixado) */
        if(!alcapaoPosicaoPraIntakeSample){
            carteiro.addOrder(intakeSuccao.IntakeSamplePositionAlcapao(), 0, "alcapao intake sample", runtime);
        }
        /*todo: Angulação do intake*/
        if(robot.teelop) {
            if(!linearEstaEstendido) {
                intakeSuccao.GotoReadyToIntakeSample();
            }
            else if(usandoInputAngulacao) {
                gerenciadorIntakeSample(carteiro, runtime);
            }
        }

        /*todo: Sugador*/
        if(usandoInputSugador){
            intakeSuccao.gerenciadorDoSugadorManual(gamepad, runtime);
        }
        if(intakeSuccaoTaNoModoIntake || samplePosicaoPraTransfer) {
            if(!usandoInputSugador) {
                carteiro.addOrder( intakeSuccao.IntakeSugar(), 0, "sugador", runtime);

            }
        }else {
            if(!usandoInputSugador) {
                carteiro.addOrder( intakeSuccao.IntakeParar(), 0, "sugador", runtime);

            }
        }
        /*todo: Sugou certo, retraiu*/
        if(samplePosicaoPraTransfer && linearEstaEstendido && !linearHorizontalMotor.isBusy) {
            carteiro.addOrder(new InstantAction(() -> underGrounSubystemStates = UnderGrounSubystemStates.INITIAL), 0, "ir pra modo Inicial", runtime);
        }


    }


  /*****************************************/
  /*********** Gerenciadores ***************/
  /*****************************************/
    public void gerenciadorIntakSpecimen(OrdersManager carteiro, double runTime){
        if(runTime < cooldown){
            return;
        }
        cooldown = runTime + 0.35;

        if(linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.RETRACTED){
            goToReadyToIntake(carteiro,runTime);
            intakeSuccao.IntakeParar();
            return;
        }
        else if(intakeSuccao.sugarAngulationStates == SugarAngulationStates.READY_TOINTAKE ){
            goToIntakeSpecimen(carteiro, runTime);
            intakeSuccao.sugador.setPower(0.7);
            return;
        }
        goToReadyToIntake(carteiro, runTime);
    }
    public void gerenciadorIntakeSample(OrdersManager carteiro, double runTime){
        if(runTime < cooldown){
            return;
        }
        if(linearHorizontalMotor.linearHorizontalInferiorState == LinearHorizontalStates.RETRACTED || linearHorizontalMotor.isBusy){
            goToReadyToIntakeSample(carteiro,runTime);
            return;
        }

        cooldown = runTime + 0.3; // 0.35

        if(intakeSuccao.sugarAngulationStates != SugarAngulationStates.INTAKE ){//
            goToIntakeSample(carteiro, runTime);
            return;
        }
        goToReadyToIntakeSample(carteiro, runTime);
    }



}
