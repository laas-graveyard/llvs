// **********************************************************************
//
// Generated by the Orbix/E IDL to Java Translator
//
// Copyright (c) 2003
// IONA Technologies, Inc.
// Waltham, MA, USA
//
// All Rights Reserved
//
// **********************************************************************

// Version: 2.0.1

package jp.go.aist.jrl.vision;

//
// IDL:CBREP_Segment:1.0
//
/***/

final public class CBREP_Segment
{
    private static final String _ob_id = "IDL:CBREP_Segment:1.0";

    public
    CBREP_Segment()
    {
    }

    public
    CBREP_Segment(int label,
                  int n,
                  int npoint,
                  float curvature,
                  float error,
                  float intensity_sigma,
                  CBREP_Point[] point,
                  byte red,
                  byte green,
                  byte blue)
    {
        this.label = label;
        this.n = n;
        this.npoint = npoint;
        this.curvature = curvature;
        this.error = error;
        this.intensity_sigma = intensity_sigma;
        this.point = point;
        this.red = red;
        this.green = green;
        this.blue = blue;
    }

    public int label;
    public int n;
    public int npoint;
    public float curvature;
    public float error;
    public float intensity_sigma;
    public CBREP_Point[] point;
    public byte red;
    public byte green;
    public byte blue;
}
