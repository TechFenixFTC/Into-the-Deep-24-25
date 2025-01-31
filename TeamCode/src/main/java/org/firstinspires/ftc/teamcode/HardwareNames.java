package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class HardwareNames {
    public static String

    /**************************************************
    *                  Rodas Mortas                   *
    **************************************************/
        par0 = "rightBack",
        par1 = "rightFront",
        perp = "leftFront",
    /**************************************************
     *                  Motores Chassi                *
     **************************************************/
        rightFront =  "rightFront",
        leftFront =   "leftFront",
        rightBack =   "rightBack",
        leftBack =    "leftBack",
    /**************************************************
    *                  Sensores                       *
    **************************************************/
        distanceSensorL      = "sensorporta3",
        distanceSensorR      = "sensorPorta2",
        colorSensor1         = "color1",
        colorSensor2         = "sensorPortaE2",

    /**************************************************
    *         Visão Computacional / Cameras          *
    **************************************************/
        webcam1              =  "Webcam 1",
        limelight            =  "limelight",
    /**************************************************
    *              DcMotors Subsistemas               *
    **************************************************/   /* Motores Vertex */
        verticalR            = "verticalr",
        verticalL            = "verticall",
        bracoGarraSuperior   = "b",
    /**************************************************
     *              Servo Subsistemas               *
     **************************************************/   /* Motores Vertex */

    horizontalSuperiorServo = "porta2c",
    horizontalInferiorServo = "porta1",
    bracoGarraSuperiorServo   = "porta1c",

    bracoGarraInferiorServo = "porta0",
    rotacaoGarraInferiorServo = "porta2",

    angulacaoGarraInferiorServo = "porta5",

    angulacaoGarraSuperiorServo = "porta0c",

    aberturaGarraInferiorServo = "porta3",

    aberturaGarraSuperiorServo = "porta3c",

    /**************************************************
     *                 Servo-motores                  *
     **************************************************/
    /**************************************************
     *                 TroughBore                  *
     **************************************************/
    horizontalSuperior    = "braco",
    getHorizontalInferior  = "braco";
}
