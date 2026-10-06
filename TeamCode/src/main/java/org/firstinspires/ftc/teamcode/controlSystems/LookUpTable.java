package org.firstinspires.ftc.teamcode.controlSystems;
import com.arcrobotics.ftclib.util.InterpLUT;

public class LookUpTable {
    InterpLUT rpm = new InterpLUT();
    InterpLUT hood = new InterpLUT();
    InterpLUT time = new InterpLUT();
    InterpLUT transferPower = new InterpLUT();
    public LookUpTable(){

        // Clamping floor
        rpm.add(-1000, 2580);
        hood.add(-1000, 0.4);
        time.add(-1000, 0.46);
        transferPower.add(-1000, 1);


        // New values from firm table
        rpm.add(42.45, 2610);
        hood.add(42.45, 0.4);
        time.add(42.45, 0.46);
        transferPower.add(42.45, 1);


        rpm.add(52.11, 2710);
        hood.add(52.11, 0.34);
        time.add(52.11, 0.51);
        transferPower.add(52.11, 1);


        rpm.add(62.5, 2810);
        hood.add(62.5, 0.3);
        time.add(62.5, 0.56);
        transferPower.add(62.5, 1);


        rpm.add(72.44, 2920);
        hood.add(72.44, 0.25);
        time.add(72.44, 0.61);
        transferPower.add(72.44, 1);


        rpm.add(85.88, 3060);
        hood.add(85.88, 0.21);
        time.add(85.88, 0.67);
        transferPower.add(85.88, 1);


        rpm.add(107.53, 3240);
        hood.add(107.53, 0.18);
        time.add(107.53, 0.74);
        transferPower.add(107.53, 1);


        transferPower.add(110, .5);

        rpm.add(124.86, 3500);
        hood.add(124.86, 0.17);
        time.add(124.86, 0.82);
        transferPower.add(124.86, 0.5);


        rpm.add(145.5, 3800);
        hood.add(145.5, 0.17);
        time.add(145.5, 0.92);
        transferPower.add(145.5, 0.5);


        rpm.add(155.2, 3900);
        hood.add(155.2, 0.17);
        time.add(155.2, 0.97);
        transferPower.add(155.2, 0.5);

        // CRI ONLY //
//
//        rpm.add(160, 4600);
//        hood.add(160, 0.16);
//        time.add(160, 0.99);
//
//        rpm.add(165, 4660);
//        hood.add(165, 0.16);
//        time.add(165, 1.01);
//
//        rpm.add(170, 4720);
//        hood.add(170, 0.16);
//        time.add(170, 1.04);
//
//        rpm.add(175, 4800);
//        hood.add(175, 0.15);
//        time.add(175, 1.06);
//
//        rpm.add(180, 4880);
//        hood.add(180, 0.15);
//        time.add(180, 1.08);
//
//        rpm.add(185, 4980);
//        hood.add(185, 0.14);
//        time.add(185, 1.11);
//
//        rpm.add(190, 5100);
//        hood.add(190, 0.13);
//        time.add(190, 1.13);
//
//        rpm.add(195, 5150);
//        hood.add(195, 0.13);
//        time.add(195, 1.15);
//
//        rpm.add(200, 5200);
//        hood.add(200, 0.13);
//        time.add(200, 1.17);


        // Clamping ceiling
        rpm.add(1000, 4340);
        hood.add(1000, 0.17);
        time.add(1000, 0.97);
        transferPower.add(1000, 0.5);

        rpm.createLUT();
        hood.createLUT();
        time.createLUT();
        transferPower.createLUT();

    }

    public double getRPM(double distance){
        return rpm.get(distance);
    }

    public double getHood(double distance){
        return hood.get(distance);
    }

    public double getTime(double distance){
        return time.get(distance);
    }

    public double getTransferPower(double distance){
        return transferPower.get(distance);
    }


}
