package org.firstinspires.ftc.teamcode.agregadoras.agregadorasSubsistemas.Superior;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.subsystems.OrdersManager;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.BracoGarra.BracoGarraSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Garra.GarraSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Horizontal.LinearHorizontalSuperior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.LinearVertical.LinearVertical;

import java.util.ArrayList;
import java.util.List;

public class SubsistemasSuperiores {

        public Telemetry telemetry;
        public LinearVertical linearVertical;

        public LinearHorizontalSuperior horizontalSuperior;
        public GarraSuperior garraSuperior;
        public BracoGarraSuperior braco;
        HardwareMap hardwaremap;





        public SubsistemasSuperiores(HardwareMap hardwareMap, Telemetry telemetry) {

            this.hardwaremap = hardwareMap;
            this.telemetry = telemetry;
            this.braco = new BracoGarraSuperior(hardwareMap,telemetry);
            this.linearVertical = new LinearVertical(hardwareMap);
            this.horizontalSuperior = new LinearHorizontalSuperior(hardwareMap);
            this.garraSuperior = new GarraSuperior(hardwareMap);
            //this.runningActions2 = new ArrayList<>();
        }

        public Action goToTransfer(OrdersManager carteiro){
            return new InstantAction(()->{
                carteiro.addOrder(braco.goToTransfer(), 0,"braco superior");
                carteiro.addOrder(garraSuperior.goToTransfer(), 0.9,"garra Superior");
            });
        }

        public Action goToOuttake(OrdersManager carteiro){
            return new InstantAction(()->{
                carteiro.addOrder(braco.goToOuttake(), 0,"braco garra superior");
                carteiro.addOrder(garraSuperior.goToOuttake(), 0.5,"fechar garra");
            });

        }

        private double getVoltage() {return hardwaremap.voltageSensor.iterator().next().getVoltage();}
        // todo: mudar função para o v5
    }


