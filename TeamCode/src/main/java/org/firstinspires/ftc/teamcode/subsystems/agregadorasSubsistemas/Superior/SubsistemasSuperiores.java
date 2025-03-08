package org.firstinspires.ftc.teamcode.subsystems.agregadorasSubsistemas.Superior;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Controller.OrdersManager;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.BracoGarra.BracoGarraSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.BracoGarra.BracoGarraSuperiorStates;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Garra.GarraSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.LinearVertical.LinearVertical;
import org.firstinspires.ftc.teamcode.subsystems.agregadorasSubsistemas.Inferior.UnderGrounSubystemStates;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.Garra;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.GarraAngulationStates;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.GarraOpeningStates;

public class SubsistemasSuperiores {

        public Telemetry telemetry;
        public LinearVertical linearVertical;
        public UpperSubsystemStates upperSubsystemStates = UpperSubsystemStates.TRANSFER;
        public GarraSuperior garraSuperior;
        public BracoGarraSuperior braco;
        HardwareMap hardwaremap;
        public ElapsedTime tempoInferiorEmTransfer = new ElapsedTime(), tempoVerticalResetado = new ElapsedTime();


        public boolean voltandoPraPegarUmaSample = false;


        public SubsistemasSuperiores(HardwareMap hardwareMap, Telemetry telemetry) {

            this.hardwaremap = hardwareMap;
            this.telemetry = telemetry;
            this.braco = new BracoGarraSuperior(hardwareMap,telemetry);
            this.linearVertical = new LinearVertical(hardwareMap);
            this.garraSuperior = new GarraSuperior(hardwareMap);
            //this.runningActions2 = new ArrayList<>();
        }
    /*****************************************/
    /*********** Funções Antigas  ************/
    /*****************************************/
        public void goToTransfer(OrdersManager carteiro, double runtime){
                carteiro.addOrder(actionGoTransfer(), 0, "subsistemas superiores", runtime);
        }
        public void goToReadyTransfer(OrdersManager carteiro, double delay, double runtime){
            carteiro.addOrder(actionGoReadyTransfer(), delay, "braco garra superior", runtime);

            //carteiro.addOrder(garraSuperior.goToTransfer(), 1.0,"garra Superior", runtime);
            //carteiro.addOrder(braco.goToTransfer(), 0.5,"braco superior", runtime);
            //carteiro.addOrder(garraSuperior.abrirGarra(),4.0,"abrir garra",runtime);

        }
        public void goToIntakeCHAMBER(OrdersManager carteiro , double runtime){
            carteiro.addOrder(braco.goToIntakeCHAMBER(),0.0,"braco superior",runtime);
            carteiro.addOrder(garraSuperior.goToIntakeSpecimen(),0.0,"garra superior",runtime);
            carteiro.addOrder(linearVertical.ElevadorGoTo(-700),0.0,"linear vertical",runtime);

        }
        public void goToInitial(OrdersManager carteiro , double runtime){
            carteiro.addOrder(braco.goToInital(),0.0,"braco superior",runtime);
            carteiro.addOrder(garraSuperior.goToIntakeSpecimen(),0.0,"garra superior",runtime);
            carteiro.addOrder(garraSuperior.abrirGarra(),0.0,"Abrir",runtime);
            carteiro.addOrder(linearVertical.ElevadorGoTo(-700),0.0,"linear vertical",runtime);

        }
        public void goToOuttakeCHAMBER(OrdersManager carteiro , double runtime){
            carteiro.addOrder(braco.goToOuttakeCHAMBER(),0.0,"braco superior",runtime);
            carteiro.addOrder(garraSuperior.goToOuttakeSpecimen(),0.0,"garra superior", runtime);
            carteiro.addOrder(linearVertical.ElevadorGoTo(LinearVertical.alturaOuttakeChamber),0.0,"linear vertical",runtime);


    }
        public void goToOuttakeBASKET(OrdersManager carteiro, double delay, double runtime){

            // upperSubsystemStates = UpperSubsystemStates.OUTTAKE;
            if (!carteiro.hasOrder("fechar garra")) {
                carteiro.addOrder(garraSuperior.fecharGarra(), delay, "fechar garra", runtime);
            }
            if (!carteiro.hasOrder("braco garra superior")) {
                carteiro.addOrder(braco.goToOuttakeBASKET(), 0.49 + delay,"braco garra superior", runtime);
            }
            if (!carteiro.hasOrder("vertical")) {
                carteiro.addOrder(linearVertical.ElevadorGoTo(3100), 0.46 + delay,"vertical", runtime);
            }
            if (!carteiro.hasOrder("garra superior")) {
                carteiro.addOrder(garraSuperior.goToOuttakeSample(), 0.9 + delay,"garra superior", runtime);
            }




        }
        public Action actionGoReadyTransfer() {
                    /*if (upperSubsystemStates == UpperSubsystemStates.OUTTAKE){// todo: criar estado gerais
                        return  new SequentialAction(
                                braco.goToReadyToTransfer(),
                                garraSuperior.goToTransfer(),
                                new MecanumDrive(hardwaremap, new Pose2d(0,0,0)).actionBuilder(new Pose2d(0,0,0)).waitSeconds(0.2).build(),
                                linearVertical.ElevadorGoTo(-300),
                                //elevador -> 200
                                new ParallelAction(

                                )
                        );
                    }*/
            Action returnedAction;
            if(linearVertical.motorR.getCurrentPosition() < 60) {
                returnedAction = new SequentialAction(
                        garraSuperior.goToTransfer(),
                        new MecanumDrive(hardwaremap, new Pose2d(0,0,0)).actionBuilder(new Pose2d(0,0,0)).waitSeconds(0.2).build(),
                        braco.goToReadyToTransfer()
                );
            }
            else {
                returnedAction = new SequentialAction(
                        garraSuperior.goToTransfer(),
                        new MecanumDrive(hardwaremap, new Pose2d(0,0,0)).actionBuilder(new Pose2d(0,0,0)).waitSeconds(0.2).build(),
                        linearVertical.ElevadorGoTo(-700),
                        braco.goToReadyToTransfer()
                );
            }
            return  returnedAction;


        }
        public Action actionGoTransfer() {
            return  new SequentialAction(

                    garraSuperior.goToTransfer(),
                    new MecanumDrive(hardwaremap, new Pose2d(0,0,0)).actionBuilder(new Pose2d(0,0,0)).waitSeconds(0.25).build(),
                    braco.goToTransfer(),
                    new MecanumDrive(hardwaremap, new Pose2d(0,0,0)).actionBuilder(new Pose2d(0,0,0)).waitSeconds(0.4).build(),
                    garraSuperior.fecharGarra()
            );
        }
        public void CorreProTransfer(OrdersManager carteiro, double runtime){
            carteiro.addOrder(actionGoTransfer(),0.0,"transfer",runtime);
        }

     /*****************************************/
     /******** Funções modo Specimen **********/
     /*****************************************/
     //todo teleop
        public void runStatesSpecimen(OrdersManager carteiro, double runtime, V5 robot, GamepadEx gamepad) {

            if(upperSubsystemStates == UpperSubsystemStates.TRANSFER || upperSubsystemStates == UpperSubsystemStates.INITIAL) {
                Initial_Specimen(carteiro, runtime, robot, gamepad);
            }

            if(upperSubsystemStates == UpperSubsystemStates.OUTTAKE) {
                Outtake_Specimen(carteiro, runtime, robot, gamepad);
            }
            if(upperSubsystemStates == UpperSubsystemStates.INTAKE_CHAMBER){
                Intake_Specimen(carteiro, runtime, robot, gamepad);
            }

        }
        public  void  Initial_Specimen(OrdersManager carteiro, double runtime, V5 robot, GamepadEx gamepad) {
        //todo okey
        //Posição Vertical
        boolean verticalEstaRetraido  = linearVertical.position <50;
        boolean verticalEstaExtendido = linearVertical.position >900;
        //Estados braço
        boolean bracoEstaNaPosicaoOuttakeSpecimen = braco.bracoGarraSuperiorState == BracoGarraSuperiorStates.OUTTAKE_CHAMBER;
        boolean bracoEstaNaPosicaoInitial = braco.bracoGarraSuperiorState == BracoGarraSuperiorStates.INITIAL;
        //Estado angulação
        boolean angulacaoEstaNaPosicaoInitialSpecimen = garraSuperior.garraAngulationState == GarraAngulationStates.INITIAL_SPECIMEN;


        double delay = 0;
        // AÇÕES DE TRANSIÇÃO Automática
        /* todo: ir para Initial quando> 1- Vertical Retraido 2- Braco no estado initial 3- Angulação no estado inicial*/
        if(verticalEstaRetraido && bracoEstaNaPosicaoInitial && angulacaoEstaNaPosicaoInitialSpecimen){
            carteiro.addOrder(new InstantAction(() -> upperSubsystemStates = UpperSubsystemStates.INITIAL), 0, "Mudar modo subsistemas superiores para initial", runtime);
        }
        // AÇÕES DO ESTADO

        //todo abre a garra se estiver em estado de outtake/
        if(bracoEstaNaPosicaoOuttakeSpecimen){
            carteiro.addOrder(garraSuperior.abrirGarra(),0,"abrirGarra",runtime);
        }

        /* todo: mudar o braco para initial */
        if(!bracoEstaNaPosicaoInitial){
            carteiro.addOrder(braco.goToInital(), 0.0,"bracoSuperior",runtime);
        }
        /* todo: mudar a Angulação  e Rotação para initial */
        if(!angulacaoEstaNaPosicaoInitialSpecimen){
            carteiro.addOrder(garraSuperior.goToInitialSpecimen(),0,"garraSuperior",runtime);
        }
        //todo: retrair o vertical/
        if(!verticalEstaRetraido){
            carteiro.addOrder(linearVertical.ElevadorGoTo(0),0,"linearVertical",runtime);
        }



    }
        public void Outtake_Specimen(OrdersManager carteiro, double runtime, V5 robot, GamepadEx gamepad){
        //todo okey
        //Posições Vertical
        boolean verticalEstaRetraido  = linearVertical.position <50;
        boolean verticalEstaNaPosicaoOuttakeSpecimen = linearVertical.position >900;
        //Estado braço
        boolean bracoEstaNaPosicaoOuttakeSpecimen = braco.bracoGarraSuperiorState == BracoGarraSuperiorStates.OUTTAKE_CHAMBER;
        //Estado angulação
        boolean angulacaoEstaNaPosicaoOuttakeSpecimen = garraSuperior.garraAngulationState == GarraAngulationStates.OUTTAKE_SPECIMEN;


        double delay = 0;
        // AÇÕES DE TRANSIÇÃO Automática
        /* todo: ir para Initial quando> 1- Vertical na posição adequada 2- Braco no estado outtake 3- Garra Superior no estado outtake*/
        if(verticalEstaNaPosicaoOuttakeSpecimen && bracoEstaNaPosicaoOuttakeSpecimen && angulacaoEstaNaPosicaoOuttakeSpecimen){
            carteiro.addOrder(new InstantAction(() -> upperSubsystemStates = UpperSubsystemStates.OUTTAKE_CHAMBER), 0, "Mudar modo subsistemas superiores para outtake Specimen", runtime);
        }
        // AÇÕES DO ESTADO
            //todo: Braco para posição de Outtake/
        if(!bracoEstaNaPosicaoOuttakeSpecimen){
            carteiro.addOrder(braco.goToOuttakeCHAMBER(), 0,"bracoSuperior",runtime);
        }
            //todo: Angulção,Abertura e Rotação para a posição de Outtake/
        if(!angulacaoEstaNaPosicaoOuttakeSpecimen){
            carteiro.addOrder(garraSuperior.goToOuttakeSpecimen(),0,"garraSuperior",runtime);
        }
            //todo: vertical para a posição de Outtake/
        if(!verticalEstaNaPosicaoOuttakeSpecimen){
            carteiro.addOrder(linearVertical.ElevadorGoTo(LinearVertical.alturaOuttakeChamber),0,"linearVertical",runtime);
        }


    }
        public void Intake_Specimen(OrdersManager carteiro, double runtime, V5 robot, GamepadEx gamepad){
            //todo okey
            //Posições Vertical
            boolean verticalEstaRetraido  = linearVertical.position <50;
            boolean verticalEstaNaPosicaoOuttakeSpecimen = linearVertical.position >900;
            //Estado braço
            boolean bracoEstaNaPosicaoIntakeSpecimen = braco.bracoGarraSuperiorState == BracoGarraSuperiorStates.INTAKE;
            //Estados Angulação
            boolean angulacaoEstaNaPosicaoIntakeSpecimen = garraSuperior.garraAngulationState == GarraAngulationStates.INTAKE_SPECIMEN;
            //Estado Abertura da garra
            boolean garraEstaHalf = garraSuperior.garraOpeningState == GarraOpeningStates.HALF;


            double delay = 0;
            // AÇÕES DE TRANSIÇÃO Automática
            /* todo: ir para Initial quando> 1- Vertical Retraido 2- Braco no estado intake 3- Garra Superior no estado intakeSpecimen*/
            if(verticalEstaRetraido && bracoEstaNaPosicaoIntakeSpecimen && angulacaoEstaNaPosicaoIntakeSpecimen){
                carteiro.addOrder(new InstantAction(() -> upperSubsystemStates = UpperSubsystemStates.INTAKE_CHAMBER), 0, "Mudar modo subsistemas superiores para intakeSpecimen", runtime);
            }
            // AÇÕES DO ESTADO
                //todo: Braco para a posição de intake/
            if(!bracoEstaNaPosicaoIntakeSpecimen){
                carteiro.addOrder(braco.goToIntakeCHAMBER(),0,"braco",runtime);
            }
                //todo: Angulação, Rotação da garra para a posição de intake/
            if(!angulacaoEstaNaPosicaoIntakeSpecimen){
                carteiro.addOrder(garraSuperior.goToIntakeSpecimen(),0,"garraSuperior",runtime);
            }
                //todo: abertura da Garra para half/
            if(!garraEstaHalf){
                carteiro.addOrder(garraSuperior.abrirGarraHalf(),0,"AbrirGarra",runtime);
            }
                //todo: Retrai o vertical/
            if(verticalEstaRetraido){
                carteiro.addOrder(linearVertical.ElevadorGoTo(-700),0,"linearVertical",runtime);
            }
        }




     /*****************************************/
     /******** Funções modo Sample ************/
     /*****************************************/
     //todo teleop
        public void runStatesSample(OrdersManager carteiro, double runtime, V5 robot, GamepadEx gamepad) {

                if(upperSubsystemStates == UpperSubsystemStates.TRANSFER || upperSubsystemStates == UpperSubsystemStates.INITIAL) {
                    Initial_ReadyToTransferSample(carteiro, runtime, robot, gamepad);
                }
                if(upperSubsystemStates == UpperSubsystemStates.OUTTAKE) {
                    OutakeHighBasket(carteiro, runtime, robot, gamepad);
                }


            }

        private void Initial_ReadyToTransferSample(OrdersManager carteiro, double runtime, V5 robot, GamepadEx gamepad) {
            int maxPositionVerticalTransfer = 78;
            boolean intakeInferiorTaProntoProTransfer   = robot.intakeInferior.underGrounSubystemStates == UnderGrounSubystemStates.TRANSFER;
            boolean intakeProntoProTransferAalgumTempo  = tempoInferiorEmTransfer.time() >= 0.500;
            boolean verticalEstaResetado                = linearVertical.motorR.getCurrentPosition() < maxPositionVerticalTransfer;
            boolean bracoTaNaPosicaoDeTransfer          = braco.bracoGarraSuperiorState ==  BracoGarraSuperiorStates.TRANSFER;
            boolean garraTaAberta                       = garraSuperior.garraOpeningState == GarraOpeningStates.OPEN;
            boolean angulacaoGarraTaPraTransfer         = garraSuperior.garraAngulationState == GarraAngulationStates.TRANSFER;
            boolean estaApertandoParaResetar            = gamepad.getButton(GamepadKeys.Button.B);
            boolean precisaTirarOSistemaDaBasket        = linearVertical.motorR.getCurrentPosition() > 2500;


           /*todo: Precisa resetar o Linear?*/
            if((!verticalEstaResetado || estaApertandoParaResetar)|| runtime <= 0.1) {
                if(!carteiro.hasOrder("resetarOvertical")) {
                    carteiro.addOrder(linearVertical.ElevadorGoTo(-800), 0, "resetarOvertical", runtime);
                }
            }
           /*todo: precisa por os servos pra transfer?*/
            if(!garraTaAberta || !angulacaoGarraTaPraTransfer) {
                Action goToTransfer;
                if(!garraTaAberta) {
                    goToTransfer = new SequentialAction(
                            garraSuperior.abrirGarra(),
                            new MecanumDrive(hardwaremap, new Pose2d(0,0,0)).actionBuilder(new Pose2d(0,0,0)).waitSeconds(0.1).build(),
                            garraSuperior.goToTransfer(),
                            new MecanumDrive(hardwaremap, new Pose2d(0,0,0)).actionBuilder(new Pose2d(0,0,0)).waitSeconds(0.2).build(),
                            braco.goToReadyToTransfer()
                    );
                }
                else{
                    goToTransfer = new SequentialAction(
                            garraSuperior.goToTransfer(),
                            new MecanumDrive(hardwaremap, new Pose2d(0,0,0)).actionBuilder(new Pose2d(0,0,0)).waitSeconds(0.2).build(),
                            braco.goToReadyToTransfer()
                    );
                }
                if(!carteiro.hasOrder("sistemas Superiores indo pra intake")) {
                carteiro.addOrder(goToTransfer, 0, "sistemas Superiores indo pra intake", runtime);
                }

            }

           /*todo: gerenciar se deve resetar o Timer que conta quanto tempo tem sample pra transfer*/
            if(!intakeInferiorTaProntoProTransfer){ tempoInferiorEmTransfer.reset(); }
           /*todo: gerenciar se deve resetar o Timer que conta quanto tempo o vertical ta resetado*/
            if(!verticalEstaResetado) { tempoVerticalResetado.reset();}

           /*todo: Adicionar Função pra mover os sistemas de servo manualmente*/

           /*todo: Adicionar Função pra abrir e fechar a garra e rotacionar ela pra posições predefinidas com apenas um botão*/

           /*todo: Adicionar Função de mover o vertical manualmente*/

           /*todo: nessa função ele vê se o  robo esta voltando pra pegar uma sample, dai verifica se o vertical ta resetado a uns 200ms e ai ele desliga a condição pro transfer acontecer*/
            if(voltandoPraPegarUmaSample && tempoVerticalResetado.time() >= 0.200){
                voltandoPraPegarUmaSample = false;
            }

            /*todo: Transicionar pro estado "outake" SE 1:Vertical ta resetado 2: tem sample pronta a algum tempo 3: servos estão na posição correta 4:NãoTaVoltandoPraPegarSample*/
            if(verticalEstaResetado && intakeProntoProTransferAalgumTempo && angulacaoGarraTaPraTransfer && garraTaAberta && !voltandoPraPegarUmaSample && runtime > 1.6){
                if (!carteiro.hasOrder("ir pra modo Outake superior")) {
                    carteiro.addOrder(new InstantAction(() -> upperSubsystemStates = UpperSubsystemStates.OUTTAKE), 0.0, "ir pra modo Outake superior", runtime);
                }
            }

        }
        private void OutakeHighBasket(OrdersManager carteiro, double runtime, V5 robot, GamepadEx gamepad) {
            int maxPositionVerticalTransfer = 78;
            int positionVerticalVerificaSeNaoPegou = 600;

            boolean temSampleIntakeInferiorAinda        = robot.intakeInferior.intakeSuccao.colorSensorSugar.colorMatcher.sampleNaPosicaoCorreta();
            boolean intakeInferiorTaProntoProTransfer   = robot.intakeInferior.underGrounSubystemStates == UnderGrounSubystemStates.TRANSFER;
            boolean verticalEstaResetado                = linearVertical.motorR.getCurrentPosition() < maxPositionVerticalTransfer;
            boolean verticalPodeVerificarSeNaoPegou     = linearVertical.motorR.getCurrentPosition() > positionVerticalVerificaSeNaoPegou;
            boolean verticalJaFoiMandadoProAlto         = LinearVertical.targetPosition > 2000;
            boolean bracoTaEmOutake                     = braco.bracoGarraSuperiorState ==  BracoGarraSuperiorStates.BASKET;
            boolean garraTaFechada                      = garraSuperior.garraOpeningState == GarraOpeningStates.CLOSED;
            boolean garraTaFechando                     = garraSuperior.garraOpeningState == GarraOpeningStates.GOING_TO_CLOSE;
            boolean angulacaoGarraTaPraTransfer         = garraSuperior.garraAngulationState == GarraAngulationStates.OUTTAKE_SAMPLE;
            boolean garraFoiAbertaEsoltouUmaSample      = garraSuperior.garraOpeningState == GarraOpeningStates.OPEN && linearVertical.motorR.getCurrentPosition() > 2500;


            /*todo: Fechar a garra Se necessário*/
            if(!garraTaFechada && !garraTaFechando && !voltandoPraPegarUmaSample && verticalEstaResetado){
                if(!carteiro.hasOrder("esperar a garra fechar")) {
                    carteiro.addOrder(garraSuperior.fecharGarraTempo(), 0, "esperar a garra fechar", runtime);
                }

            }
            /*todo: Mandar Subir o Linear quando a garra pegar o sample*/
            if(!verticalJaFoiMandadoProAlto && garraTaFechada && !voltandoPraPegarUmaSample){
                carteiro.addOrder(linearVertical.ElevadorGoTo(3100), 0, "Sobe o LinearVertical", runtime);
            }
            /*todo: Mover o Braço pra Posição De Outake*/
            if(!bracoTaEmOutake && verticalPodeVerificarSeNaoPegou && !temSampleIntakeInferiorAinda && !voltandoPraPegarUmaSample){
                if(!carteiro.hasOrder("braco vai pra outake")){
                    carteiro.addOrder(braco.goToOuttakeBASKET(), 0, "braco vai pra outake", runtime);
                }
            }
            /*todo: Mover a Garra pra Posição De Outake*/
            if(!angulacaoGarraTaPraTransfer && verticalPodeVerificarSeNaoPegou && !temSampleIntakeInferiorAinda && !voltandoPraPegarUmaSample){
                if(!carteiro.hasOrder("garra vai pra outake")){
                    carteiro.addOrder(garraSuperior.goToOuttakeSample(), 0, "garra vai pra outake", runtime);
                }
            }

            /*todo: Verificar se pegou errado e a sample continua no intake*/
            if(verticalJaFoiMandadoProAlto && verticalPodeVerificarSeNaoPegou && temSampleIntakeInferiorAinda){
                voltandoPraPegarUmaSample = true;
            }
            /*todo: Voltar pro Intake pra pegar a sample que tentou pegar e não conseguiu*/
            if(voltandoPraPegarUmaSample) {
                if(!carteiro.hasOrder("superior pra transfer")) {
                    tempoInferiorEmTransfer.reset();
                    carteiro.addOrder( new InstantAction(() -> upperSubsystemStates = UpperSubsystemStates.INITIAL), 0, "superior pra transfer", runtime);
                }
            }

            /*todo: gerenciar se deve resetar o Timer que conta quanto tempo tem sample pra transfer*/
            if(!intakeInferiorTaProntoProTransfer){ tempoInferiorEmTransfer.reset(); }
            /*todo: Adicionar Função pra mover os sistemas de servo manualmente*/

            /*todo: Adicionar Função pra abrir e fechar a garra e rotacionar ela pra posições predefinidas com apenas um botão*/

            /*todo: Adicionar Função de mover o vertical manualmente*/
            /*
            //tempoInferiorEmTransfer.reset();
            if(LinearVertical.targetPosition < 1800 &&  (LinearVertical.targetPosition > -100) && linearVertical.motorR.getCurrentPosition() < 100) {
                goToOuttakeBASKET(carteiro, 0, runtime);
            }
            if(linearVertical.motorR.getCurrentPosition() > 500 && temSampleIntakeInferiorAinda && LinearVertical.targetPosition > 500) {
                if(!carteiro.hasOrder("descendo o linear e indo pra initial")) {
                    //carteiro.addOrder(actionGoReadyTransfer(), 0, "descendo o linear e indo pra initial", runtime);

                }
                if(!carteiro.hasOrder("superior pra transfer")) {
                    carteiro.addOrder( new InstantAction(() -> upperSubsystemStates = UpperSubsystemStates.INITIAL), 0.3, "superior pra transfer", runtime);


                }

            }
             */

        }

     //todo autonomo
        public void runStatesSampleAutonomo(OrdersManager carteiro, double runtime, V5 robot) {

            if(upperSubsystemStates == UpperSubsystemStates.OUTTAKE) {
                OutakeHighBasketAutonomo(carteiro, runtime, robot);

            }
            if(upperSubsystemStates == UpperSubsystemStates.TRANSFER || upperSubsystemStates == UpperSubsystemStates.INITIAL) {
                Initial_ReadyToTransferSampleAutonomo(carteiro, runtime, robot);
            }



        }

        private void Initial_ReadyToTransferSampleAutonomo(OrdersManager carteiro, double runtime, V5 robot) {
            int maxPositionVerticalTransfer = 78;
            boolean intakeInferiorTaProntoProTransfer   = robot.intakeInferior.underGrounSubystemStates == UnderGrounSubystemStates.TRANSFER;
            boolean intakeProntoProTransferAalgumTempo  = tempoInferiorEmTransfer.time() >= 0.200;
            boolean verticalEstaResetado                = linearVertical.motorR.getCurrentPosition() < maxPositionVerticalTransfer;
            boolean bracoTaNaPosicaoDeTransfer          = braco.bracoGarraSuperiorState ==  BracoGarraSuperiorStates.TRANSFER;
            boolean garraTaAberta                       = garraSuperior.garraOpeningState == GarraOpeningStates.OPEN;
            boolean angulacaoGarraTaPraTransfer         = garraSuperior.garraAngulationState == GarraAngulationStates.TRANSFER;
            boolean precisaTirarOSistemaDaBasket        = linearVertical.motorR.getCurrentPosition() > 2500;


            /*todo: Precisa resetar o Linear?*/
            if(!verticalEstaResetado || runtime <= 0.1) {
                if(!carteiro.hasOrder("resetarOvertical") && !LinearVertical.isBusy && !verticalEstaResetado) {
                    carteiro.addOrder(linearVertical.ElevadorGoTo(-800), 0, "resetarOvertical", runtime);
                }
            }
            /*todo: precisa por os servos pra transfer?*/
            if(!garraTaAberta || !angulacaoGarraTaPraTransfer) {
                Action goToTransfer;
                if(!garraTaAberta) {
                    goToTransfer = new SequentialAction(
                            garraSuperior.abrirGarra(),
                            new MecanumDrive(hardwaremap, new Pose2d(0,0,0)).actionBuilder(new Pose2d(0,0,0)).waitSeconds(0.1).build(),
                            garraSuperior.goToTransfer(),
                            new MecanumDrive(hardwaremap, new Pose2d(0,0,0)).actionBuilder(new Pose2d(0,0,0)).waitSeconds(0.2).build(),
                            braco.goToReadyToTransfer()
                    );
                }
                else{
                    goToTransfer = new SequentialAction(
                            garraSuperior.goToTransfer(),
                            new MecanumDrive(hardwaremap, new Pose2d(0,0,0)).actionBuilder(new Pose2d(0,0,0)).waitSeconds(0.2).build(),
                            braco.goToReadyToTransfer()
                    );
                }
                if(!carteiro.hasOrder("sistemas Superiores indo pra intake")) {
                    carteiro.addOrder(goToTransfer, 0, "sistemas Superiores indo pra intake", runtime);
                }

                /*todo: Transicionar pro estado "outake" SE 1:Vertical ta resetado 2: tem sample pronta a algum tempo 3: servos estão na posição correta 4:NãoTaVoltandoPraPegarSample*/
                if(verticalEstaResetado && intakeProntoProTransferAalgumTempo && angulacaoGarraTaPraTransfer && garraTaAberta && !voltandoPraPegarUmaSample && runtime > 1.6){
                    if (!carteiro.hasOrder("ir pra modo Outake superior")) {
                        carteiro.addOrder(new InstantAction(() -> upperSubsystemStates = UpperSubsystemStates.OUTTAKE), 0.0, "ir pra modo Outake superior", runtime);
                    }
                }

            }

            /*todo: gerenciar se deve resetar o Timer que conta quanto tempo tem sample pra transfer*/
            if(!intakeInferiorTaProntoProTransfer){ tempoInferiorEmTransfer.reset(); }
            /*todo: gerenciar se deve resetar o Timer que conta quanto tempo tem sample pra transfer*/
            if(!verticalEstaResetado) { tempoVerticalResetado.reset();}

            /*todo: Adicionar Função pra mover os sistemas de servo manualmente*/

            /*todo: Adicionar Função pra abrir e fechar a garra e rotacionar ela pra posições predefinidas com apenas um botão*/

            /*todo: Adicionar Função de mover o vertical manualmente*/

            /*todo: nessa função ele vê se o  robo esta voltando pra pegar uma sample, dai verifica se o vertical ta resetado a uns 200ms e ai ele desliga a condição pro transfer acontecer*/
            if(voltandoPraPegarUmaSample && tempoVerticalResetado.time() >= 0.200){
                voltandoPraPegarUmaSample = false;
            }

        }
        private void OutakeHighBasketAutonomo(OrdersManager carteiro, double runtime, V5 robot) {
            int maxPositionVerticalTransfer = 78;
            int positionVerticalVerificaSeNaoPegou = 200;
            int positionVerticalPraAbrirAGarra = 2800;

            boolean verticalTaNaAlturaParaAbrirGarra = linearVertical.motorR.getCurrentPosition() > positionVerticalPraAbrirAGarra;
            boolean temSampleIntakeInferiorAinda        = robot.intakeInferior.intakeSuccao.colorSensorSugar.colorMatcher.sampleNaPosicaoCorreta();
            boolean verticalEstaResetado                = linearVertical.motorR.getCurrentPosition() < maxPositionVerticalTransfer;
            boolean verticalPodeVerificarSeNaoPegou     = linearVertical.motorR.getCurrentPosition() > positionVerticalVerificaSeNaoPegou;
            boolean verticalJaFoiMandadoProAlto         = LinearVertical.targetPosition > 2000;
            boolean verticalTaProAlto                   = linearVertical.motorR.getCurrentPosition() > 3000;
            boolean bracoTaEmOutake                     = braco.bracoGarraSuperiorState ==  BracoGarraSuperiorStates.BASKET;
            boolean garraTaFechada                      = garraSuperior.garraOpeningState == GarraOpeningStates.CLOSED;
            boolean garraTaFechando                     = garraSuperior.garraOpeningState == GarraOpeningStates.GOING_TO_CLOSE;
            boolean angulacaoGarraTaPraTransfer         = garraSuperior.garraAngulationState == GarraAngulationStates.OUTTAKE_SAMPLE;
            boolean garraFoiAbertaEsoltouUmaSample      = garraSuperior.garraOpeningState == GarraOpeningStates.OPEN && linearVertical.motorR.getCurrentPosition() > 2500;

            /*todo: Fechar a garra Se necessário*/
            //if(!garraTaFechada && !garraTaFechando && !voltandoPraPegarUmaSample && !verticalPodeVerificarSeNaoPegou){
               // if(!carteiro.hasOrder("esperar a garra fechar")) {
                //carteiro.addOrder(garraSuperior.fecharGarraTempo(), 0, "esperar a garra fechar", runtime);
               // }

         //   }
            linearVertical.PIDF();
            if(!garraTaFechada && !garraTaFechando && !voltandoPraPegarUmaSample && verticalEstaResetado){
                if(!carteiro.hasOrder("esperar a garra fechar")) {
                    carteiro.addOrder(garraSuperior.fecharGarraTempo(), 0, "esperar a garra fechar", runtime);
                }

            }
            /*todo: Mandar Subir o Linear quando a garra pegar o sample*/
            if(!verticalJaFoiMandadoProAlto && garraTaFechada && !voltandoPraPegarUmaSample){
                carteiro.addOrder(linearVertical.ElevadorGoTo(3100), 0, "Sobe o LinearVertical", runtime);
            }
            /*todo: Mover o Braço pra Posição De Outake*/
            if(!bracoTaEmOutake && verticalPodeVerificarSeNaoPegou && !temSampleIntakeInferiorAinda && !voltandoPraPegarUmaSample){
                if(!carteiro.hasOrder("braco vai pra outake")){
                    carteiro.addOrder(braco.goToOuttakeBASKET(), 0, "braco vai pra outake", runtime);
                }
            }
            /*todo: Mover a Garra pra Posição De Outake*/
            if(!angulacaoGarraTaPraTransfer && verticalPodeVerificarSeNaoPegou && !temSampleIntakeInferiorAinda && !voltandoPraPegarUmaSample){
                if(!carteiro.hasOrder("garra vai pra outake")){
                    carteiro.addOrder(garraSuperior.goToOuttakeAngulation(), 0, "garra vai pra outake", runtime);
                }
            }

            if(verticalTaProAlto && garraTaFechada){
                if(!carteiro.hasOrder("esperar garra abrir")){
                    carteiro.addOrder(garraSuperior.abrirGarraTempo(), 0,"esperar garra abrir", runtime);
                }

            }
            /*todo: Verificar se pegou errado e a sample continua no intake*/
            if(verticalJaFoiMandadoProAlto && verticalPodeVerificarSeNaoPegou && temSampleIntakeInferiorAinda){
                voltandoPraPegarUmaSample = true;
            }
            /*todo: Voltar pro Intake pra pegar a sample que tentou pegar e não conseguiu*/
            if(voltandoPraPegarUmaSample) {
                if(!carteiro.hasOrder("superior pra transfer")) {
                    tempoInferiorEmTransfer.reset();
                    carteiro.addOrder( new InstantAction(() -> upperSubsystemStates = UpperSubsystemStates.INITIAL), 0, "superior pra transfer", runtime);
                }
            }

        }

        public void monitorEstadosAutonomo(TelemetryPacket telemetry){
            telemetry.addLine(String.format("🏗️ Estado Geral Superiores: %s", upperSubsystemStates));
            telemetry.addLine(String.format("🕰️ Tempo Inferior em Transferência: %s", tempoInferiorEmTransfer.time()));
            telemetry.addLine(String.format("♻️ Voltando Para Pegar Sample: %s", voltandoPraPegarUmaSample));
            telemetry.addLine("Linear pos:"+linearVertical.motorR.getCurrentPosition()+" | Linear Alvo"+LinearVertical.targetPosition + " | 👷IsBusy: "+ LinearVertical.isBusy + " | Tempo ElevadorGoTo: "+linearVertical.timeElevadorGoTo);
            telemetry.addLine("Corrente Linear Vertical: "+ linearVertical.motorR.getCurrent(CurrentUnit.AMPS) + " | " + "Limite Corrente Topo: " + LinearVertical.valorDeSurtoDeCorrenteTopo);
            telemetry.put("Corrente Linear Vertical", linearVertical.motorR.getCurrent(CurrentUnit.AMPS));

            garraSuperior.monitorAutonomo(telemetry);

        }


        private double getVoltage() {return hardwaremap.voltageSensor.iterator().next().getVoltage();}
        // todo: mudar função para o v5
    }


