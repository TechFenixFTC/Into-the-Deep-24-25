package org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.BracoGarra;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareNames;

import java.util.HashMap;

public class BracoGarraInferior {
    public Servo bracoGarraInferior;
    BracoGarraInferiorStates bracoGarraInferiorState = BracoGarraInferiorStates.INITIAL;
    HashMap<BracoGarraInferiorStates, Double> mapBraco = new HashMap<>();

    public BracoGarraInferior(HardwareMap hardwareMap){

        bracoGarraInferior = hardwareMap.get(Servo.class, HardwareNames.bracoGarraInferiorServo);
        mapBraco.put(BracoGarraInferiorStates.READY_TOINTAKE, 0.145);
        mapBraco.put(BracoGarraInferiorStates.INITIAL, 0.314);
        mapBraco.put(BracoGarraInferiorStates.TRASNFER, 0.314);
        mapBraco.put(BracoGarraInferiorStates.INTAKE,0.074);

    }
    public Action goToReadytoIntake(){
        bracoGarraInferiorState = BracoGarraInferiorStates.READY_TOINTAKE;

        return new InstantAction(() -> {
            bracoGarraInferior.setPosition(mapBraco.get(bracoGarraInferiorState));
        });
    }
    public Action goToInitial(){
        bracoGarraInferiorState = BracoGarraInferiorStates.INITIAL;

        return new InstantAction(() -> {
            bracoGarraInferior.setPosition(mapBraco.get(bracoGarraInferiorState));
        });
    }
    public Action goToIntake(){
       bracoGarraInferiorState = BracoGarraInferiorStates.INTAKE;

        return new InstantAction(() -> {
           bracoGarraInferior.setPosition(mapBraco.get(bracoGarraInferiorState));
        });
    }

    public Action goToTransfer(){
       bracoGarraInferiorState = BracoGarraInferiorStates.TRASNFER;

        return new InstantAction(() -> {
           bracoGarraInferior.setPosition(mapBraco.get(bracoGarraInferiorState));
        });



    }


}
