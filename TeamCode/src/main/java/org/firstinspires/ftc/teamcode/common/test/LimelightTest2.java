package org.firstinspires.ftc.teamcode.common.test;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.BracoGarra.BracoGarraSuperior.distance;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.HardwareNames;

import java.util.List;

@TeleOp (name = "Limelight Teste")
public class LimelightTest2 extends OpMode {
    Limelight3A limelight;
    LLResultTypes.FiducialResult fiducialResult;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, HardwareNames.limelight);
        limelight.setPollRateHz(100); // Isso define quantas vezes pedimos dados ao Limelight (100 vezes por segundo)
        limelight.start(); // Isso diz ao Limelight para começar a procurar!
        limelight.pipelineSwitch(0); // Muda para o pipeline número 0

    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();


        if (result != null && result.isValid()) {
            double tx = result.getTx(); // Quão longe o alvo está para a esquerda ou direita (graus)
            double ty = result.getTy(); // Quão longe o alvo está para cima ou para baixo (graus)
            double ta = result.getTa(); // Quão grande o alvo parece (0%-100% da imagem)
            result.getPipelineIndex();
            telemetry.addData("Alvo X", tx);
            telemetry.addData("Alvo Y", ty);
            telemetry.addData("Área do Alvo", ta);
        } else {
            telemetry.addData("Limelight", "Sem Alvos");
        }






        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // O numerous ID do fiducial
            LLResultTypes.FiducialResult detection = null;
           // double x = detection.getTargetXDegrees(); // Onde está (esquerda-direita)
           // double y = detection.getTargetYDegrees(); // Onde está (cima-baixo)
            YawPitchRollAngles StrafeDistance_3D = fiducial.getRobotPoseTargetSpace().getOrientation();
            double distance = 2;

         //   telemetry.addData("Fiducial " + id, "está a " + distance + " metros de distância" + "x:"+x+"y:"+y);
        }
        //fiducialResult.getRobotPoseTargetSpace(); // Pose do robô relativa ao Sistema de Coordenadas do AprilTag (Mais Útil)
        //fiducialResult.getCameraPoseTargetSpace(); // Pose da câmera relativa ao AprilTag (útil)
        //fiducialResult.getRobotPoseFieldSpace(); // Pose do robô no sistema de coordenadas do campo baseado apenas nesta tag (útil)
        //fiducialResult.getTargetPoseCameraSpace(); // Pose do AprilTag no sistema de coordenadas da câmera (não muito útil)
        //fiducialResult.getTargetPoseRobotSpace(); // Pose do AprilTag no sistema de coordenadas do robô (não muito útil)

        if(gamepad1.a){
            limelight.captureSnapshot("auto_pov_10s");
        }
    }





}