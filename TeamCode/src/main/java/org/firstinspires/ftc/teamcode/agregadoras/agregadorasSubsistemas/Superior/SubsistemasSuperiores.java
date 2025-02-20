package org.firstinspires.ftc.teamcode.agregadoras.agregadorasSubsistemas.Superior;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.agregadoras.agregadorasRobo.V5;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.OrdersManager;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.BracoGarra.BracoGarraSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Garra.GarraSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.LinearVertical.LinearVertical;

public class SubsistemasSuperiores {

        public Telemetry telemetry;
        public LinearVertical linearVertical;
        public UpperSubsystemStates upperSubsystemStates = UpperSubsystemStates.TRANSFER;
        public GarraSuperior garraSuperior;
        public BracoGarraSuperior braco;
        HardwareMap hardwaremap;





        public SubsistemasSuperiores(HardwareMap hardwareMap, Telemetry telemetry) {

            this.hardwaremap = hardwareMap;
            this.telemetry = telemetry;
            this.braco = new BracoGarraSuperior(hardwareMap,telemetry);
            this.linearVertical = new LinearVertical(hardwareMap);
            this.garraSuperior = new GarraSuperior(hardwareMap);
            //this.runningActions2 = new ArrayList<>();
        }

        public void goToTransfer(OrdersManager carteiro, double runtime){
                carteiro.addOrder(actionGoTrasnfer(), 0, "subsistemas superiores", runtime);

                /*carteiro.addOrder(linearVertical.ElevadorGoTo(0),0.0,"subir ",runtime);
                carteiro.addOrder(garraSuperior.goToTransfer(), 1.0,"garra Superior", runtime);
                carteiro.addOrder(braco.goToTransfer(), 0.5,"braco superior", runtime);
                carteiro.addOrder(garraSuperior.abrirGarra(),4.0,"abrir garra",runtime);
                carteiro.addOrder(linearVertical.ElevadorGoTo(300),4,"linear vertical",runtime);
                carteiro.addOrder(garraSuperior.fecharGarra(),4.0,"abrir garra",runtime);*/

        }

        public Action Test(){
            return new SequentialAction(

                    linearVertical.ElevadorGoTo(700),

                     new MecanumDrive(hardwaremap, new Pose2d(0,0,0)).actionBuilder(new Pose2d(0,0,0)).waitSeconds(4).build(),
                    linearVertical.ElevadorGoTo(370)


            );
        }
        public void goToReadyTransfer(OrdersManager carteiro,double runtime){
            carteiro.addOrder(actionGoReadyTrasnfer(), 0, "braco garra superior", runtime);

            //carteiro.addOrder(garraSuperior.goToTransfer(), 1.0,"garra Superior", runtime);
            //carteiro.addOrder(braco.goToTransfer(), 0.5,"braco superior", runtime);
            //carteiro.addOrder(garraSuperior.abrirGarra(),4.0,"abrir garra",runtime);

        }


        public void goToIntakeCHAMBER(OrdersManager carteiro , double runtime){
            carteiro.addOrder(braco.goToIntakeCHAMBER(),0.0,"braco superior",runtime);
            carteiro.addOrder(garraSuperior.goToIntakeSpecimen(),0.0,"garra superior",runtime);
            carteiro.addOrder(linearVertical.ElevadorGoTo(-100),0.0,"linear vertical",runtime);

        }
        public void goToInitial(OrdersManager carteiro , double runtime){
            carteiro.addOrder(braco.goToInital(),0.0,"braco superior",runtime);
            carteiro.addOrder(garraSuperior.goToIntakeSpecimen(),0.0,"garra superior",runtime);
            carteiro.addOrder(garraSuperior.abrirGarra(),0.0,"Abrir",runtime);
            carteiro.addOrder(linearVertical.ElevadorGoTo(0),0.0,"linear vertical",runtime);

        }
        public Action goToIntakeChamber2() {
            return new ParallelAction(
                    braco.goToIntakeCHAMBER(),
                    garraSuperior.goToIntakeSpecimen(),
                    garraSuperior.abrirGarra(),
                    linearVertical.ElevadorGoTo(0)
            );

        }

        public void goToReadOuttakeCHAMBER(OrdersManager carteiro , double runtime ){
            carteiro.addOrder(braco.goToReadOuttakeCHAMBER(),0.0,"braco superior", runtime);
            carteiro.addOrder(garraSuperior.goToReadOuttakeSpecimen(),0.0,"garra inferior",runtime);
            carteiro.addOrder(linearVertical.ElevadorGoTo(330),0.0,"linear vertical",runtime);
    }
        public void goEconomiadeBateria(OrdersManager carteiro , double runtime){
        carteiro.addOrder(braco.goToOuttakeCHAMBER(),0.0,"braco superior",runtime);
        carteiro.addOrder(garraSuperior.goToOuttakeSpecimen(),0.0,"garra superior", runtime);
        carteiro.addOrder(linearVertical.ElevadorGoTo(700),0.0,"linear vertical",runtime);


    }
        public void goToOuttakeCHAMBER(OrdersManager carteiro , double runtime){
            carteiro.addOrder(braco.goToOuttakeCHAMBER(),0.0,"braco superior",runtime);
            carteiro.addOrder(garraSuperior.goToOuttakeSpecimen(),0.0,"garra superior", runtime);
            carteiro.addOrder(linearVertical.ElevadorGoTo(820),0.0,"linear vertical",runtime);


    }
        public void goToOuttakeTransfer(OrdersManager carteiro , double runtime, double delay){
            carteiro.addOrder(garraSuperior.abrirGarra(),0.0 + delay,"garra abrir",runtime);
            carteiro.addOrder(linearVertical.ElevadorGoTo(170),0.6 + delay,"linear vertical",runtime);
            carteiro.addOrder(braco.goToTransfer(),1 + delay,"braco", runtime);
            carteiro.addOrder(garraSuperior.goToTransfer(),0.0 + delay, "angulação", runtime);
            carteiro.addOrder(garraSuperior.fecharGarra(),1.5 + delay,"garra",runtime);

            carteiro.addOrder(linearVertical.ElevadorGoTo(550),1.9 + delay,"linear vertical outtake",runtime);
            carteiro.addOrder(braco.goToOuttakeCHAMBER(),2.0 + delay,"braco superior",runtime);
            carteiro.addOrder(garraSuperior.goToOuttakeEjecting(),4.0 + delay,"garra superior", runtime);
            carteiro.addOrder(garraSuperior.abrirGarra(),5.0 + delay,"abrir garra", runtime);


    }
        public void goToOuttakeEjecting(OrdersManager carteiro , double runtime){
        carteiro.addOrder(braco.goToOuttakeCHAMBER(),0.0 ,"braco superior",runtime);
        carteiro.addOrder(garraSuperior.goToOuttakeEjecting(),0.0,"garra superior", runtime);
        carteiro.addOrder(garraSuperior.abrirGarra(),0.5,"abrir garra", runtime);



    }

        public void goToOuttakeCHAMBERSUBIR(OrdersManager carteiro , double runtime){
        carteiro.addOrder(braco.goToOuttakeCHAMBER(),0.0,"braco superior",runtime);
        carteiro.addOrder(garraSuperior.goToOuttakeSpecimen(),0.0,"garra superior", runtime);
        carteiro.addOrder(linearVertical.ElevadorGoTo(1400),0.0,"linear vertical", runtime);
        carteiro.addOrder(garraSuperior.abrirGarra(),1,"abrido",runtime);


    }


        public Action goToOuttakeCHAMBER(){
            return new ParallelAction(
                    braco.goToOuttakeCHAMBER(),
                    garraSuperior.goToOuttakeSpecimen(),
                    new MecanumDrive(hardwaremap, new Pose2d(0,0,0)).actionBuilder(new Pose2d(0,0,0)).waitSeconds(0.2).build(),
                    linearVertical.ElevadorGoTo(330)
            );
    }
        public void goToOuttakeBASKET(OrdersManager carteiro, double runtime){
           upperSubsystemStates = UpperSubsystemStates.OUTTAKE;
           carteiro.addOrder(garraSuperior.angularTransfer(),0, "angulacao garra", runtime);
           carteiro.addOrder(braco.goToOuttakeBASKET(), 0.4,"braco garra superior", runtime);
           carteiro.addOrder(linearVertical.ElevadorGoTo(2900), 0.3,"vertical", runtime);
           carteiro.addOrder(garraSuperior.goToOuttakeSample(), 0.9,"garra superior", runtime);


        }




        /*****************************************/
        /************** Actions ******************/
        /*****************************************/

        public Action actionGoReadyTrasnfer() {
            if (upperSubsystemStates == UpperSubsystemStates.OUTTAKE){// todo: criar estado gerais
                return  new SequentialAction(
                        braco.goToReadyToTransfer(),
                        new MecanumDrive(hardwaremap, new Pose2d(0,0,0)).actionBuilder(new Pose2d(0,0,0)).waitSeconds(0.2).build(),
                        linearVertical.ElevadorGoTo(-200),
                        //elevador -> 200
                        new ParallelAction(
                                garraSuperior.goToReadyToTransfer()
                        )
                );
            }
            return  new SequentialAction(
                    linearVertical.ElevadorGoTo(-300),
                    //elevador -> 200
                    new ParallelAction(
                            braco.goToReadyToTransfer(),
                            garraSuperior.goToTransfer()
                    )
            );
        }
        public Action actionGoTrasnfer() {
        return  new SequentialAction(
                //linearVertical.ElevadorGoTo(-300),

                //elevador -> 200
                new ParallelAction(
                        braco.goToTransfer(),
                        garraSuperior.goToTransfer()
                ),
                new MecanumDrive(hardwaremap, new Pose2d(0,0,0)).actionBuilder(new Pose2d(0,0,0)).waitSeconds(0.4).build(),
                garraSuperior.fecharGarra()
        );
    }
        public Action goToOuttake2(){
            return new ParallelAction(
                    braco.goToOuttakeBASKET(),
                    garraSuperior.goToOuttakeSample(),
                    new MecanumDrive(hardwaremap, new Pose2d(0,0,0)).actionBuilder(new Pose2d(0,0,0)).waitSeconds(0.2).build(),
                    linearVertical.ElevadorGoTo(1350)
            );
        }




        private double getVoltage() {return hardwaremap.voltageSensor.iterator().next().getVoltage();}
        // todo: mudar função para o v5
    }


