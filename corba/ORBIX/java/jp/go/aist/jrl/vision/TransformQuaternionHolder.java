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
// IDL:TransformQuaternion:1.0
//
final public class TransformQuaternionHolder implements org.omg.CORBA.portable.Streamable
{
    public TransformQuaternion value;

    public
    TransformQuaternionHolder()
    {
    }

    public
    TransformQuaternionHolder(TransformQuaternion initial)
    {
        value = initial;
    }

    public void
    _read(org.omg.CORBA.portable.InputStream in)
    {
        value = TransformQuaternionHelper.read(in);
    }

    public void
    _write(org.omg.CORBA.portable.OutputStream out)
    {
        TransformQuaternionHelper.write(out, value);
    }

    public org.omg.CORBA.TypeCode
    _type()
    {
        return TransformQuaternionHelper.type();
    }
}
