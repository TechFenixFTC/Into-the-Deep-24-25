package org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Garra;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.GarraOpeningStates;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.GarraRotationStates;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.GarraV4;

import java.util.HashMap;

public class GarraInferior extends GarraV4 {
    public Servo rotacaoGarraInferiorServo;
    public Servo angulacaoGarraInferiorServo;
    public Servo aberturaGarraInferiorServo;
    boolean continous = false;
    GarraRotationStates garraRotationState = GarraRotationStates.PARALELA;
    GarraOpeningStates garraOpeningState = GarraOpeningStates.OPEN;
    GarraAngulationInferiorStates garraAngulationState = GarraAngulationInferiorStates.TRANSFER;
    HashMap<GarraRotationStates, Double> mapRotation = new HashMap<>();
    HashMap< GarraOpeningStates, Double> mapOpening = new HashMap<>();
    HashMap<GarraAngulationInferiorStates, Double> mapAngulation = new HashMap<>();
    public GarraInferior(HardwareMap hardwareMap) {

        super(hardwareMap);
        rotacaoGarraInferiorServo = hardwareMap.get(Servo.class, HardwareNames.rotacaoGarraInferiorServo);
        angulacaoGarraInferiorServo = hardwareMap.get(Servo.class, HardwareNames.angulacaoGarraInferiorServo);
        aberturaGarraInferiorServo = hardwareMap.get(Servo.class, HardwareNames.aberturaGarraInferiorServo);

        mapRotation.put(GarraRotationStates.PARALELA, 0.47);
        mapRotation.put(GarraRotationStates.PERPENDICULAR, 0.832);


        mapOpening.put(GarraOpeningStates.OPEN, 0.548);
        mapOpening.put(GarraOpeningStates.CLOSED, 1.0);

        mapAngulation.put(GarraAngulationInferiorStates.INTAKE, 0.893);
        mapAngulation.put(GarraAngulationInferiorStates.TRANSFER,0.838);
        mapAngulation.put(GarraAngulationInferiorStates.READYTO_INTAKE, 1.0);
    }
    public Action goToReadytoIntake(){
        garraRotationState = GarraRotationStates.PARALELA;
        garraOpeningState = GarraOpeningStates.OPEN;
        garraAngulationState = GarraAngulationInferiorStates.READYTO_INTAKE;
        return new InstantAction(() -> {
            aberturaGarraInferiorServo.setPosition(mapOpening.get(garraOpeningState));
            angulacaoGarraInferiorServo.setPosition(mapAngulation.get(garraAngulationState));
            rotacaoGarraInferiorServo.setPosition(mapRotation.get(garraRotationState));
        });
    }
    public Action goToAbrirGarra(){
        garraOpeningState = GarraOpeningStates.OPEN;
        return new InstantAction(()-> {
            aberturaGarraInferiorServo.setPosition(mapOpening.get(garraOpeningState));
        });
    }
    public Action goToIntake(){
        garraRotationState = GarraRotationStates.PARALELA;
        garraOpeningState = GarraOpeningStates.CLOSED;
        garraAngulationState = GarraAngulationInferiorStates.INTAKE;
        return new InstantAction(() -> {
            aberturaGarraInferiorServo.setPosition(mapOpening.get(garraOpeningState));
            angulacaoGarraInferiorServo.setPosition(mapAngulation.get(garraAngulationState));
            rotacaoGarraInferiorServo.setPosition(mapRotation.get(garraRotationState));
        });
    }
    public Action goToTransfer(){
        garraRotationState = GarraRotationStates.PARALELA;
        garraOpeningState = GarraOpeningStates.CLOSED;
        garraAngulationState = GarraAngulationInferiorStates.TRANSFER;

        return new InstantAction(() -> {
            aberturaGarraInferiorServo.setPosition(mapOpening.get(garraOpeningState));
            angulacaoGarraInferiorServo.setPosition(mapAngulation.get(garraAngulationState));
            rotacaoGarraInferiorServo.setPosition(mapRotation.get(garraRotationState));

        });



    }
    public void monitor(Telemetry telemetry) {
        this.monitor(telemetry,"Inferior");
    }
}
