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
// IDL:CBREP_PointSeq:1.0
//
final public class CBREP_PointSeqHolder implements org.omg.CORBA.portable.Streamable
{
    public CBREP_Point[] value;

    public
    CBREP_PointSeqHolder()
    {
    }

    public
    CBREP_PointSeqHolder(CBREP_Point[] initial)
    {
        value = initial;
    }

    public void
    _read(org.omg.CORBA.portable.InputStream in)
    {
        value = CBREP_PointSeqHelper.read(in);
    }

    public void
    _write(org.omg.CORBA.portable.OutputStream out)
    {
        CBREP_PointSeqHelper.write(out, value);
    }

    public org.omg.CORBA.TypeCode
    _type()
    {
        return CBREP_PointSeqHelper.type();
    }
}
