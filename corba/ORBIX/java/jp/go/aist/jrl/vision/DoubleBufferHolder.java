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
// IDL:DoubleBuffer:1.0
//
final public class DoubleBufferHolder implements org.omg.CORBA.portable.Streamable
{
    public double[] value;

    public
    DoubleBufferHolder()
    {
    }

    public
    DoubleBufferHolder(double[] initial)
    {
        value = initial;
    }

    public void
    _read(org.omg.CORBA.portable.InputStream in)
    {
        value = DoubleBufferHelper.read(in);
    }

    public void
    _write(org.omg.CORBA.portable.OutputStream out)
    {
        DoubleBufferHelper.write(out, value);
    }

    public org.omg.CORBA.TypeCode
    _type()
    {
        return DoubleBufferHelper.type();
    }
}
