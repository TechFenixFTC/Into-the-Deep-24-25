package org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.Garra;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Garra.GarraAngulationInferiorStates;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.GarraOpeningStates;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.GarraRotationStates;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.GarraV4;

import java.util.HashMap;


public class GarraSuperior extends GarraV4 {

    public Servo aberturaGarraSuperiorServo;
    public Servo  angulacaoGarraSuperiorServo;
    GarraRotationStates garraRotationState = GarraRotationStates.PARALELA;
    GarraOpeningStates garraOpeningState = GarraOpeningStates.OPEN;
    GarraAngulationSuperiorStates garraAngulationState = GarraAngulationSuperiorStates.TRANSFER;
    HashMap<GarraRotationStates, Double> mapRotation = new HashMap<>();
    HashMap< GarraOpeningStates, Double> mapOpening = new HashMap<>();
    HashMap<GarraAngulationSuperiorStates, Double> mapAngulation = new HashMap<>();

    public GarraSuperior(HardwareMap hardwareMap) {
        super(hardwareMap);

        angulacaoGarraSuperiorServo = hardwareMap.get(Servo.class, HardwareNames.angulacaoGarraInferiorServo);
        aberturaGarraSuperiorServo = hardwareMap.get(Servo.class, HardwareNames.aberturaGarraInferiorServo);

        mapRotation.put(GarraRotationStates.PARALELA, 0.47);
        mapRotation.put(GarraRotationStates.PERPENDICULAR, 0.832);


        mapOpening.put(GarraOpeningStates.OPEN, 0.548);
        mapOpening.put(GarraOpeningStates.CLOSED, 1.0);

        mapAngulation.put(GarraAngulationSuperiorStates.INTAKE, 0.893);
        mapAngulation.put(GarraAngulationSuperiorStates.TRANSFER,0.2);
        mapAngulation.put(GarraAngulationSuperiorStates.OUTTAKE_CHAMBER,0.467);
        mapAngulation.put(GarraAngulationSuperiorStates.INTAKE_CHAMBER,0.467);
    }

    public Action goToTransfer(){
            garraOpeningState = GarraOpeningStates.CLOSED;
            garraAngulationState = GarraAngulationSuperiorStates.TRANSFER;
            return new InstantAction(() -> {
                aberturaGarraSuperiorServo.setPosition(mapOpening.get(garraOpeningState));
                angulacaoGarraSuperiorServo.setPosition(mapAngulation.get(garraAngulationState));
            });
    }
    public Action goToOuttake(){
        garraOpeningState = GarraOpeningStates.CLOSED;
        garraAngulationState = GarraAngulationSuperiorStates.TRANSFER;
        return new InstantAction(()->{
            aberturaGarraSuperiorServo.setPosition(mapOpening.get(garraOpeningState));
        });
    }

    public void monitor(Telemetry telemetry) {
        this.monitor(telemetry,"Superior");
    }
}
