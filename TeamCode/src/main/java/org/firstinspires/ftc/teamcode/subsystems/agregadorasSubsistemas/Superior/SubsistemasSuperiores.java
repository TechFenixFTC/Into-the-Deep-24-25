package org.firstinspires.ftc.teamcode.subsystems.agregadorasSubsistemas.Superior;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Controller.OrdersManager;
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
                carteiro.addOrder(actionGoTransfer(), 0, "subsistemas superiores", runtime);
        }

        public void goToReadyTransfer(OrdersManager carteiro,double runtime){
            carteiro.addOrder(actionGoReadyTransfer(), 0, "braco garra superior", runtime);

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

        public void goToOuttakeBASKET(OrdersManager carteiro, double runtime){
           upperSubsystemStates = UpperSubsystemStates.OUTTAKE;
           carteiro.addOrder(garraSuperior.angularTransfer(),0, "angulacao garra", runtime);
           carteiro.addOrder(braco.goToOuttakeBASKET(), 0.4,"braco garra superior", runtime);
           carteiro.addOrder(linearVertical.ElevadorGoTo(3100), 0.3,"vertical", runtime);
           carteiro.addOrder(garraSuperior.goToOuttakeSample(), 0.9,"garra superior", runtime);


        }


        /*****************************************/
        /************** Actions ******************/
        /*****************************************/

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
            }else {
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




        private double getVoltage() {return hardwaremap.voltageSensor.iterator().next().getVoltage();}
        // todo: mudar função para o v5
    }


