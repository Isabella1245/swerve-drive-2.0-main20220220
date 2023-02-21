package frc.robot;

public interface Constants{
    
    //dimensions of robot in inches
    public final double L = 33.125;
    public final double W = 18.5;

    //PIDF variables 
    //the difference between PID and PIDF:
    //PID has proportional, integral, and derivative controller actions
    //PIDF has proportional, integral, and derivative with first-order filter on derivative term
    public final double kP = 2.25;
    public final double kI = 0.0;
    public final double kD = 50;
    public final double kF = 0.0;

    //quadrature encoder ticks per rotation
    public final int quadCountsPerRotation = 1640;
    public final int analogCountsPerRotation = 2700;

    //Turn motor CAN ID
    // *** im not quite sure what the IDs are for ***
    public final int FLTid = 2; //2
    public final int FRTid = 3; //3
    public final int BRTid = 5; //5
    public final int BLTid = 7; //7

    //IDs for drive motors
    public final int FRDid = 4;
    public final int FLDid = 1;
    public final int BRDid = 6;
    public final int BLDid = 8;

    // Analog Encoder ID (i think they are refering to turning encoders)
    public final int FRTencoderID = 2;
    public final int FLTencoderID = 3;
    public final int BRTencoderID = 0;
    public final int BLTencoderID = 1;

    // Offset of encoders to make them face forwards
    public final int FRTencoderOffset = 0;
    public final int FLTencoderOffset = 0;
    public final int BRTencoderOffset = 0;
    public final int BLTencoderOffset = 0;

}

//that is all that was included in this module
