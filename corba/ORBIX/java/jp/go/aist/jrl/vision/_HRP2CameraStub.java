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
// IDL:HRP2Camera:1.0
//
public class _HRP2CameraStub extends org.omg.CORBA.portable.ObjectImpl
                             implements HRP2Camera
{
    private static final String[] _ob_ids_ =
    {
        "IDL:HRP2Camera:1.0",
    };

    public String[]
    _ids()
    {
        return _ob_ids_;
    }

    //
    // IDL:HRP2Camera/GetCameraParameter:1.0
    //
    public jp.go.aist.jrl.vision.HRP2CameraPackage.CameraParameter
    GetCameraParameter()
    {
        org.omg.CORBA.portable.Delegate _ob_delegate = _get_delegate();
        while(true)
        {
            synchronized(_ob_delegate)
            {
                org.omg.CORBA.portable.OutputStream out = null;
                org.omg.CORBA.portable.InputStream in = null;
                try
                {
                    out = _request("GetCameraParameter", true);
                    in = _invoke(out);
                    jp.go.aist.jrl.vision.HRP2CameraPackage.CameraParameter _ob_r = jp.go.aist.jrl.vision.HRP2CameraPackage.CameraParameterHelper.read(in);
                    return _ob_r;
                }
                catch(org.omg.CORBA.portable.RemarshalException _ob_ex)
                {
                    continue;
                }
                catch(org.omg.CORBA.portable.ApplicationException _ob_aex)
                {
                    final String _ob_id = _ob_aex.getId();
                    in = _ob_aex.getInputStream();

                    throw new org.omg.CORBA.UNKNOWN("Unexpected User Exception: " + _ob_id);
                }
            }
        }
    }

    //
    // IDL:HRP2Camera/GetIntrinsicParameters:1.0
    //
    public jp.go.aist.jrl.vision.HRP2CameraPackage.IntrinsicParameters
    GetIntrinsicParameters()
    {
        org.omg.CORBA.portable.Delegate _ob_delegate = _get_delegate();
        while(true)
        {
            synchronized(_ob_delegate)
            {
                org.omg.CORBA.portable.OutputStream out = null;
                org.omg.CORBA.portable.InputStream in = null;
                try
                {
                    out = _request("GetIntrinsicParameters", true);
                    in = _invoke(out);
                    jp.go.aist.jrl.vision.HRP2CameraPackage.IntrinsicParameters _ob_r = jp.go.aist.jrl.vision.HRP2CameraPackage.IntrinsicParametersHelper.read(in);
                    return _ob_r;
                }
                catch(org.omg.CORBA.portable.RemarshalException _ob_ex)
                {
                    continue;
                }
                catch(org.omg.CORBA.portable.ApplicationException _ob_aex)
                {
                    final String _ob_id = _ob_aex.getId();
                    in = _ob_aex.getInputStream();

                    throw new org.omg.CORBA.UNKNOWN("Unexpected User Exception: " + _ob_id);
                }
            }
        }
    }

    //
    // IDL:HRP2Camera/SetAcquisitionSize:1.0
    //
    public int
    SetAcquisitionSize(int _ob_a0,
                       int _ob_a1)
    {
        org.omg.CORBA.portable.Delegate _ob_delegate = _get_delegate();
        while(true)
        {
            synchronized(_ob_delegate)
            {
                org.omg.CORBA.portable.OutputStream out = null;
                org.omg.CORBA.portable.InputStream in = null;
                try
                {
                    out = _request("SetAcquisitionSize", true);
                    out.write_long(_ob_a0);
                    out.write_long(_ob_a1);
                    in = _invoke(out);
                    int _ob_r = in.read_long();
                    return _ob_r;
                }
                catch(org.omg.CORBA.portable.RemarshalException _ob_ex)
                {
                    continue;
                }
                catch(org.omg.CORBA.portable.ApplicationException _ob_aex)
                {
                    final String _ob_id = _ob_aex.getId();
                    in = _ob_aex.getInputStream();

                    throw new org.omg.CORBA.UNKNOWN("Unexpected User Exception: " + _ob_id);
                }
            }
        }
    }

    //
    // IDL:HRP2Camera/GetProjectiveParameters:1.0
    //
    public jp.go.aist.jrl.vision.HRP2CameraPackage.ProjectiveParameters
    GetProjectiveParameters()
    {
        org.omg.CORBA.portable.Delegate _ob_delegate = _get_delegate();
        while(true)
        {
            synchronized(_ob_delegate)
            {
                org.omg.CORBA.portable.OutputStream out = null;
                org.omg.CORBA.portable.InputStream in = null;
                try
                {
                    out = _request("GetProjectiveParameters", true);
                    in = _invoke(out);
                    jp.go.aist.jrl.vision.HRP2CameraPackage.ProjectiveParameters _ob_r = jp.go.aist.jrl.vision.HRP2CameraPackage.ProjectiveParametersHelper.read(in);
                    return _ob_r;
                }
                catch(org.omg.CORBA.portable.RemarshalException _ob_ex)
                {
                    continue;
                }
                catch(org.omg.CORBA.portable.ApplicationException _ob_aex)
                {
                    final String _ob_id = _ob_aex.getId();
                    in = _ob_aex.getInputStream();

                    throw new org.omg.CORBA.UNKNOWN("Unexpected User Exception: " + _ob_id);
                }
            }
        }
    }
}
