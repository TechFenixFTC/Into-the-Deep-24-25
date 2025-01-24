package org.firstinspires.ftc.teamcode.agregadoras.agregadorasSubsistemas.Inferior;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.OrdersManager;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.BracoGarra.BracoGarraInferior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Garra.GarraInferior;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Horizontal.LinearHorizontalInferior;

import java.util.ArrayList;
import java.util.List;

public class SubsistemasInferiores {

    public Telemetry telemetry;
    public UnderGrounSubystemStates underGrounSubystemStates = UnderGrounSubystemStates.INITIAL;
    public GarraInferior garraInferior;
    public BracoGarraInferior bracoGarraInferior;
    public LinearHorizontalInferior horizontalInferior;
    public List<Action> runningActions = new ArrayList<>();
    public OrdersManager carteiro =new OrdersManager(telemetry);
    HardwareMap hardwaremap;
    public SubsistemasInferiores(HardwareMap hardwareMap, Telemetry telemetry) {

        this.hardwaremap = hardwareMap;
        this.telemetry = telemetry;
        this.horizontalInferior = new LinearHorizontalInferior(hardwareMap);
        this.bracoGarraInferior = new BracoGarraInferior(hardwareMap);
        this.garraInferior = new GarraInferior(hardwareMap);
        this.runningActions = new ArrayList<>();
    }
    private double getVoltage() { return hardwaremap.voltageSensor.iterator().next().getVoltage(); }

    public Action goToIntake(){
        return new InstantAction(() -> {
            carteiro.addOrder(horizontalInferior.goToExtended(),0, "horizontal inferior");
            carteiro.addOrder(bracoGarraInferior.goToIntake(), 0.1, "braco garra inferior");
            carteiro.addOrder(garraInferior.goToIntake(),0.2,"garra inferior");
        });
    }

    public Action goToTransfer(){
        return new InstantAction(()->{
                    carteiro.addOrder(bracoGarraInferior.goToInitial(), 0,"braco garra inferior");
                    carteiro.addOrder(garraInferior.goToTransfer(), 0, "garra Inferior");
                    carteiro.addOrder(horizontalInferior.goToRetracted(),0.12, "horizontal inferior");
                    carteiro.addOrder(bracoGarraInferior.goToTransfer(), 1.0, "braco garra inferior");
        });
    }

    public Action goToReadyToIntake(){
        return new InstantAction(() -> {
            carteiro.addOrder(horizontalInferior.goToExtended(),0,"horizontal inferior");
            carteiro.addOrder(bracoGarraInferior.goToReadytoIntake(), 0.1, "braco garra inferior");
            carteiro.addOrder(garraInferior.goToReadytoIntake(), 0, "garra inferior");
        });
    }

    public Action goToReadytoIntakeNoHorizontal(){return new InstantAction(() -> {carteiro.addOrder(garraInferior.goToReadytoIntake(), 0, "garra inferior");});}

}
