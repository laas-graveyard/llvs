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
// IDL:SceneObject:1.0
//
/***/

final public class SceneObject
{
    private static final String _ob_id = "IDL:SceneObject:1.0";

    public
    SceneObject()
    {
    }

    public
    SceneObject(SceneFeature[] Features,
                DoubleMatrix xv,
                DoubleMatrix Pxx,
                int NoFeatures,
                int NoSelected)
    {
        this.Features = Features;
        this.xv = xv;
        this.Pxx = Pxx;
        this.NoFeatures = NoFeatures;
        this.NoSelected = NoSelected;
    }

    public SceneFeature[] Features;
    public DoubleMatrix xv;
    public DoubleMatrix Pxx;
    public int NoFeatures;
    public int NoSelected;
}
