package org.firstinspires.ftc.teamcode.common.controls;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Sugar.IntakeSuccao;
import org.firstinspires.ftc.teamcode.common.configSample.BlueSample;
import org.firstinspires.ftc.teamcode.common.configSample.RedSample;
import org.firstinspires.ftc.teamcode.common.configSample.YellowSample;

import java.util.ArrayList;
import java.util.List;

@Config
public class MatchColor {

    public MatchColor(){
        time.reset();
    }

    double[] red60mm = {RedSample.r60mmR, RedSample.r60mmG, RedSample.r60mmB};
    double[] red150mm = {RedSample.r150mmR, RedSample.r150mmG, RedSample.r150mmB};

    double[] blue60mm = {BlueSample.b60mmR, BlueSample.b60mmG, BlueSample.b60mmB};
    double[] blue150mm = {BlueSample.b150mmR, BlueSample.b150mmG, BlueSample.b150mmB};

    double[] yellow60mm = {YellowSample.y60mmR, YellowSample.y60mmG, YellowSample.y60mmB};
    double[] yellow150mm = {YellowSample.y150mmR, YellowSample.y150mmG,YellowSample.y150mmB};

    ElapsedTime time = new ElapsedTime();


    int somaRed;
    int somaGreen;
    int somaAlpha;
    int somaBlue;

    int green;
    int red;
    int blue;
    int alpha;

    private List<Integer> allRed = new ArrayList<>();
    private List<Integer> allBlues = new ArrayList<>();
    private List<Integer> allGreen = new ArrayList<>();
    private List<Integer> allAlpha = new ArrayList<>();

    public int media;
    boolean haveSample = false;
    public int c;

    double treshold = 0.64;

    public int getBlue(int currentBlue){
        somaBlue = 1;
        double time2 = time.seconds();

        if(time2 >= 0.1) {
            if (allBlues.size() > 0) {
                for (int blue : allBlues) {
                    somaBlue += blue;
                }
                blue = (somaBlue / allBlues.size());

                time.reset();
                return blue;
            }
        }
        else if(time2 < 0.1){
            allBlues.add(currentBlue);
        }
        return  225000;
    }
    public int getAlpha(int currentAlpha){
        somaAlpha = 1;
        double time2 = time.seconds();
        if(time2 >= 0.1) {
            if (allAlpha.size() > 0) {
                for (int alpha : allAlpha) {
                    somaAlpha += alpha;
                }
                alpha = (somaAlpha / allAlpha.size());

                time.reset();
                return alpha;
            }
        }
        else if(time2 < 0.1){
            allAlpha.add(currentAlpha);
        }

        return  224000;
    }

    public int getGreen(int currentGreen){
        somaGreen = 1;
        double time2 = time.seconds();
        if(time2 >= 0.1){
            if (allGreen.size() > 0) {
                for (int green : allGreen) {
                    somaGreen += green;
                }
                green = (somaGreen / allGreen.size());

                time.reset();
                return green;
            }
        }
        else if(time2 < 0.1){
            allGreen.add(currentGreen);
        }
        return  223000;

    }
    public int getRed(int currentRed){
        somaRed = 1;
        double time2 = time.seconds();
        if(time2 >= 0.3){
            if (allBlues.size() > 0) {
                for (int red : allRed) {
                    somaRed += red;
                }
                red = (somaRed / allRed.size());

                time.reset();
                return red;
            }
        }
        else if(time2 < 0.1){
            allRed.add(currentRed);
        }
        return  222000;
    }
    public boolean isRed( IntakeSuccao intakeSuccao){

        if(getRed(intakeSuccao.colorSensorSugar.getRed()) > 4000 && getRed(intakeSuccao.colorSensorSugar.getRed()) < 5000 &&
                getBlue(intakeSuccao.colorSensorSugar.getBlue()) < 1500 &&
                getAlpha(intakeSuccao.colorSensorSugar.getAlpha()) < 2800)
        {
            haveSample = true;
            return true;
        }
        haveSample = false;
        return false;
    }
    public boolean isBlue(IntakeSuccao intakeSuccao){

        if(getBlue(intakeSuccao.colorSensorSugar.getBlue()) > BlueSample.b60mmB &&  getRed(intakeSuccao.colorSensorSugar.getBlue()) < 700){
            haveSample = true;
            return true;
        }
        haveSample = false;
        return false;
    }
    public boolean isYellow( IntakeSuccao intakeSuccao){

        if(getGreen(intakeSuccao.colorSensorSugar.getGreen()) > 4000 && getAlpha(intakeSuccao.colorSensorSugar.getAlpha()) > 2500){
            haveSample = true;
            return true;
        }
        haveSample = false;
        return false;
    }


    /*public String colorSample(double currentRed , double currentGreen,  double currentBlue, IntakeSuccao intakeSuccao){

        /*if(intakeSuccao.colorSensorSugar.getDistance() >= 0.6 && intakeSuccao.colorSensorSugar.getDistance()  <=  0.87) {
            media = (blue60mm[2] / currentBlue);
            if (media > treshold) {
                haveSample = true;
                return "BLUE";
            }

            media = (1 - (yellow60mm[0] / currentRed) +  (1 - (yellow60mm[1] / currentGreen)) + (1 - (yellow60mm[2] / currentBlue))) / 3;
            if (media > treshold) {
                haveSample = true;
                return "YELLOW";
            }

            media = (red60mm[0] / currentRed);
            if (media < treshold) {
                haveSample = true;
                return "RED";
            }
        }
        if(intakeSuccao.colorSensorSugar.getDistance() > 0.87 && intakeSuccao.colorSensorSugar.getDistance() <= 1.54){
            media = ((red150mm[0] / currentRed) + (red150mm[1] / currentGreen) + (red150mm[2] / currentBlue)) / 3;
            if (media > treshold) {
                haveSample = true;
                return "RED";
            }

            media = ((yellow150mm[0] / currentRed) + (yellow150mm[1] / currentGreen) + (yellow150mm[2] / currentBlue)) / 3;
            if (media > treshold) {
                haveSample = true;
                return "YELLOW";
            }

            media = ((blue150mm[0] / currentRed) + (blue150mm[1] / currentGreen) + (blue150mm[2] / currentBlue)) / 3;
            if (media > treshold) {
                haveSample = true;
                return "BLUE";
            }
        }
        haveSample = false;
        return "EMPTY";
    }*/

    public boolean verifyPositionSampleToTransfer() {

        return haveSample;
    }
}
