package org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.BracoGarra;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareNames;

import java.util.HashMap;

@Config
public class BracoGarraSuperior {
    int ID = 0;
    boolean isBusy = false;
    public static boolean monitor;
    public double servoBracoSuperiorPosition =0.55555;
    public Servo bracoGarraSuperiorServo;
    public BracoGarraSuperiorStates bracoGarraSuperiorState = BracoGarraSuperiorStates.READYTO_TRANSFER;
    public HashMap<BracoGarraSuperiorStates, Double> mapBracoSuperior = new HashMap<>();
    public static  double   maxAcc = 2700, maxVelocity  = 4400, distance = 0, tempo = 0;
    public BracoGarraSuperior(HardwareMap hardwareMap, Telemetry telemetry) {
        bracoGarraSuperiorServo = hardwareMap.get(Servo.class, HardwareNames.bracoGarraSuperiorServo);

        mapBracoSuperior.put(BracoGarraSuperiorStates.BASKET, 0.652);///todo okey
        mapBracoSuperior.put(BracoGarraSuperiorStates.INITIAL,0.233);//todo rever as posições
        mapBracoSuperior.put(BracoGarraSuperiorStates.READYTO_TRANSFER, 0.14);//todo rever as posições
        mapBracoSuperior.put(BracoGarraSuperiorStates.TRANSFER, 0.0);///todo okey
        mapBracoSuperior.put(BracoGarraSuperiorStates.OUTTAKE_EJECTING, 1.0);

        mapBracoSuperior.put(BracoGarraSuperiorStates.INTAKE,0.147);//todo okey
        mapBracoSuperior.put(BracoGarraSuperiorStates.OUTTAKE,0.87);//todo okey

        mapBracoSuperior.put(BracoGarraSuperiorStates.READY_OUTTAKE,0.78);//todo rever as posições



    }


    public Action goToTransfer(){//todo okey
        final int IDaction = 3;
        return new InstantAction(()->{
            ID = IDaction;
            bracoGarraSuperiorState = BracoGarraSuperiorStates.TRANSFER;
            servoBracoSuperiorPosition = mapBracoSuperior.get(bracoGarraSuperiorState);
            bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
        });
    }

    public Action goToReadyToTransfer(){//todo okey
        final int IDaction = 2;
        return new InstantAction(()->{
            ID = IDaction;
            bracoGarraSuperiorState = BracoGarraSuperiorStates.READYTO_TRANSFER;
            servoBracoSuperiorPosition = mapBracoSuperior.get(bracoGarraSuperiorState);
            bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
        });
    }
    public Action timeActionBracoGarraSuperior(BracoGarraSuperiorStates targetState){
        return new Action(){
            private final int IDaction = 1;
            boolean FIRST = true;
            double delay = 0.2;
            ElapsedTime time = new ElapsedTime();
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if(FIRST) {
                    time.reset();
                    FIRST = false;
                    isBusy = true;
                    ID = IDaction;
                }

                if(ID != IDaction){
                    return false;
                }

                bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(targetState));
                if(delay <= time.time()){
                    bracoGarraSuperiorState = targetState;
                    isBusy = false;
                    return false;
                }
                return true;
            }
        };

    }
    public Action goToOuttakeBASKET(){//todo okey
        final int IDaction = 4;
        return new InstantAction(()->{
            ID = IDaction;
            bracoGarraSuperiorState = BracoGarraSuperiorStates.BASKET;
            servoBracoSuperiorPosition = mapBracoSuperior.get(bracoGarraSuperiorState);
            bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
        });
    }
    public Action goToReadOuttakeCHAMBER(){
        final int IDaction = 5;
        return new InstantAction(() -> {
            ID = IDaction;
            bracoGarraSuperiorState = BracoGarraSuperiorStates.READY_OUTTAKE;
            servoBracoSuperiorPosition = mapBracoSuperior.get(bracoGarraSuperiorState);
            bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
        });
    }

   public Action goToIntakeCHAMBER(){//todo okey
       final int IDaction = 6;
        return new InstantAction(() ->{
            ID = IDaction;
            bracoGarraSuperiorState = BracoGarraSuperiorStates.INTAKE;
            servoBracoSuperiorPosition = mapBracoSuperior.get(bracoGarraSuperiorState);
            bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
        });
   }

    public Action goToInital(){//todo okey
        final int IDaction = 7;
        return new InstantAction(() -> {
            ID = IDaction;
            bracoGarraSuperiorState = BracoGarraSuperiorStates.INITIAL;
            servoBracoSuperiorPosition = mapBracoSuperior.get(bracoGarraSuperiorState);
            bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
        });
    }

   public Action goToOuttakeCHAMBER(){
       final int IDaction = 8;
        return new InstantAction(() ->{//todo okey
            ID = IDaction;
            bracoGarraSuperiorState =BracoGarraSuperiorStates.OUTTAKE;
            servoBracoSuperiorPosition = mapBracoSuperior.get(bracoGarraSuperiorState);
            bracoGarraSuperiorServo.setPosition(mapBracoSuperior.get(bracoGarraSuperiorState));
        });
   }

   public void upSetPoint(double increase) { // todo: corrigido!
       double position = Range.clip(bracoGarraSuperiorServo.getPosition() + (increase * 0.005),0,1);
       bracoGarraSuperiorServo.setPosition(position);
    }
   public void downSetPoint(double decrease) { // todo: corrigido!
        double position = Range.clip(bracoGarraSuperiorServo.getPosition() + (decrease * 0.005),0,1);
        bracoGarraSuperiorServo.setPosition(position);
    }



    /*public void monitor(Telemetry telemetry) {
        if (monitor) {
            telemetry.addLine("==============================");
            telemetry.addLine("  TELEMETRIA DO BRAÇO DA GARRA");
            telemetry.addLine("======fdo braco: ",this.getAngle());
            telemetry.addData("-alvo: ",targetAngle);
            //telemetry.addData("",);

        }
    }*/


}
