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

package jp.go.aist.jrl.vision.LowLevelVisionSystemPackage;

//
// IDL:LowLevelVisionSystem/VisionSensorException:1.0
//
final public class VisionSensorExceptionHolder implements org.omg.CORBA.portable.Streamable
{
    public VisionSensorException value;

    public
    VisionSensorExceptionHolder()
    {
    }

    public
    VisionSensorExceptionHolder(VisionSensorException initial)
    {
        value = initial;
    }

    public void
    _read(org.omg.CORBA.portable.InputStream in)
    {
        value = VisionSensorExceptionHelper.read(in);
    }

    public void
    _write(org.omg.CORBA.portable.OutputStream out)
    {
        VisionSensorExceptionHelper.write(out, value);
    }

    public org.omg.CORBA.TypeCode
    _type()
    {
        return VisionSensorExceptionHelper.type();
    }
}
