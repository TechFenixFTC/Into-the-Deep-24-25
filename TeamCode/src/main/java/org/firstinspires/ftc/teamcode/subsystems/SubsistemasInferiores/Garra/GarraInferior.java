package org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Garra;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.GarraOpeningStates;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.GarraRotationStates;
import org.firstinspires.ftc.teamcode.subsystems.common.Garra.GarraV4;

import java.util.HashMap;

public class GarraInferior extends GarraV4 {

    Servo bracoGarraInferiorServo;
    Servo rotacaoGarraInferiorServo;
    Servo angulacaoGarraInferiorServo;
    Servo aberturaGarraInferiorServo;
    boolean continous = false;

    GarraRotationStates garraRotationState = GarraRotationStates.PARALELA;
    GarraOpeningStates garraOpeningState = GarraOpeningStates.OPEN;
    BracoGarraInferiorStates bracoGarraInferiorState = BracoGarraInferiorStates.INITIAL;
    GarraAngulationStates garraAngulationState = GarraAngulationStates.RETO;

    HashMap<GarraRotationStates, Double> mapRotation = new HashMap<>();
    HashMap<BracoGarraInferiorStates, Double> mapBraco = new HashMap<>();
    HashMap< GarraOpeningStates, Double> mapOpening = new HashMap<>();
    HashMap<GarraAngulationStates, Double> mapAngulation = new HashMap<>();



    public GarraInferior(HardwareMap hardwareMap) {

        super(hardwareMap);
        bracoGarraInferiorServo = hardwareMap.get(Servo.class, HardwareNames.bracoGarraInferiorServo);
        rotacaoGarraInferiorServo = hardwareMap.get(Servo.class, HardwareNames.rotacaoGarraInferiorServo);
        angulacaoGarraInferiorServo = hardwareMap.get(Servo.class, HardwareNames.angulacaoGarraInferiorServo);
        aberturaGarraInferiorServo = hardwareMap.get(Servo.class, HardwareNames.aberturaGarraInferiorServo);

        mapRotation.put(GarraRotationStates.PARALELA, 0.616);
        mapRotation.put(GarraRotationStates.PERPENDICULAR, 0.298);

        mapBraco.put(BracoGarraInferiorStates.INTAKE, 0.78);
        mapBraco.put(BracoGarraInferiorStates.INITIAL, 0.60);
        mapBraco.put(BracoGarraInferiorStates.TRASNFER, 0.50);

        mapOpening.put(GarraOpeningStates.OPEN, 0.02);
        mapOpening.put(GarraOpeningStates.CLOSED, 0.773);

        mapAngulation.put(GarraAngulationStates.RETO, 0.386);



    }

    public Action goToIntake(double runTime){
        garraRotationState = GarraRotationStates.PARALELA;
        garraOpeningState = GarraOpeningStates.OPEN;
        bracoGarraInferiorState = BracoGarraInferiorStates.INTAKE;
        garraAngulationState = GarraAngulationStates.RETO;

        return  new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                aberturaGarraInferiorServo.setPosition(mapOpening.get(garraOpeningState));
                angulacaoGarraInferiorServo.setPosition(mapAngulation.get(angulacaoGarraInferiorServo));
                bracoGarraInferiorServo.setPosition(mapBraco.get(bracoGarraInferiorState));
                rotacaoGarraInferiorServo.setPosition(mapRotation.get(garraRotationState));

                return false;

            }
        };
    }
   /* public Action goToState(double runTime){

         return new Action() {
             @Override
             public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                 aberturaGarraInferiorServo.setPosition(mapOpening.get(garraOpeningStates));
                 angulacaoGarraInferiorServo.setPosition(mapAngulation.get(angulacaoGarraInferiorServo));
                 bracoGarraInferiorServo.setPosition(mapBraco.get(bracoGarraInferiorStates));
                 rotacaoGarraInferiorServo.setPosition(mapRotation.get(garraRotationStates));

                 return true;
             }
         };
    }*/

    public void monitor(Telemetry telemetry) {
        this.monitor(telemetry,"Inferior");
    }
}
