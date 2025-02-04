package org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.BracoGarra;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Garra.GarraRotationInferiorStates;

import java.util.HashMap;

@Config
public class BracoGarraSuperior {
    public static boolean monitor;
    public double servoBracoSuperiorPosition =0.55555;
    public Servo bracoGarraSuperiorServo;
    public BracoGarraSuperiorStates bracoGarraSuperiorState = BracoGarraSuperiorStates.READYTO_TRANSFER;
    public HashMap<BracoGarraSuperiorStates, Double> mapBracoSuperior = new HashMap<>();
    public static  double   maxAcc = 2700, maxVelocity  = 4400, distance = 0, tempo = 0;
    public BracoGarraSuperior(HardwareMap hardwareMap, Telemetry telemetry) {
        bracoGarraSuperiorServo = hardwareMap.get(Servo.class, HardwareNames.bracoGarraSuperiorServo);

        mapBracoSuperior.put(BracoGarraSuperiorStates.BASKET, 0.25);//todo rever as posições
        mapBracoSuperior.put(BracoGarraSuperiorStates.INITIAL,0.403);//todo rever as posições
        mapBracoSuperior.put(BracoGarraSuperiorStates.READYTO_TRANSFER, 0.14);//todo rever as posições
        mapBracoSuperior.put(BracoGarraSuperiorStates.TRANSFER, 0.022);//todo rever as posições

        mapBracoSuperior.put(BracoGarraSuperiorStates.INTAKE,0.111);//todo rever as posições
        mapBracoSuperior.put(BracoGarraSuperiorStates.OUTTAKE,0.78);//todo rever as posições
        mapBracoSuperior.put(BracoGarraSuperiorStates.READY_OUTTAKE,0.78);//todo rever as posições



    }


    public Action goToTransfer(){//todo okey

        return new InstantAction(()->{
            bracoGarraSuperiorState = BracoGarraSuperiorStates.TRANSFER;
            servoBracoSuperiorPosition = mapBracoSuperior.get(bracoGarraSuperiorState);
            bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
        });
    }
    public Action goToReadyToTransfer(){//todo okey

        return new InstantAction(()->{
            bracoGarraSuperiorState = BracoGarraSuperiorStates.READYTO_TRANSFER;
            servoBracoSuperiorPosition = mapBracoSuperior.get(bracoGarraSuperiorState);
            bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
        });
    }
    public Action goToOuttake(){//todo okey

        return new InstantAction(()->{
            bracoGarraSuperiorState = BracoGarraSuperiorStates.BASKET;
            servoBracoSuperiorPosition = mapBracoSuperior.get(bracoGarraSuperiorState);
            bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
        });
    }
    public Action goToInital(){//todo okey

        return new InstantAction(() -> {
            bracoGarraSuperiorState = BracoGarraSuperiorStates.INITIAL;
            servoBracoSuperiorPosition = mapBracoSuperior.get(bracoGarraSuperiorState);
            bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
        });
    }

   public Action goToIntakeCHAMBER(){//todo okey
        return new InstantAction(() ->{
            bracoGarraSuperiorState = BracoGarraSuperiorStates.INTAKE;
            servoBracoSuperiorPosition = mapBracoSuperior.get(bracoGarraSuperiorState);
            bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
        });
   }
   public Action goToReadOuttakeCHAMBER(){
        return new InstantAction(() -> {
            bracoGarraSuperiorState = BracoGarraSuperiorStates.READY_OUTTAKE;
            servoBracoSuperiorPosition = mapBracoSuperior.get(bracoGarraSuperiorState);
            bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
        });
   }

   public Action goToOuttakeCHAMBER(){
        return new InstantAction(() ->{//todo okey
            bracoGarraSuperiorState =BracoGarraSuperiorStates.OUTTAKE;
            servoBracoSuperiorPosition = mapBracoSuperior.get(bracoGarraSuperiorState);
            bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
        });
   }




    /*public void monitor(Telemetry telemetry) {
        if (monitor) {
            telemetry.addLine("==============================");
            telemetry.addLine("  TELEMETRIA DO BRAÇO DA GARRA");
            telemetry.addLine("===============================");
            telemetry.addData("-Angulo do braco: ",this.getAngle());
            telemetry.addData("-alvo: ",targetAngle);
            //telemetry.addData("",);

        }
    }*/


}
