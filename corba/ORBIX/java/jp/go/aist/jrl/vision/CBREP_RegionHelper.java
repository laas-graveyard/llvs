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
// IDL:CBREP_Region:1.0
//
final public class CBREP_RegionHelper
{
    public static void
    insert(org.omg.CORBA.Any any, CBREP_Region val)
    {
        org.omg.CORBA.portable.OutputStream out = any.create_output_stream();
        write(out, val);
        any.read_value(out.create_input_stream(), type());
    }

    public static CBREP_Region
    extract(org.omg.CORBA.Any any)
    {
        if(any.type().equivalent(type()))
            return read(any.create_input_stream());
        else
            throw new org.omg.CORBA.BAD_OPERATION();
    }

    private static org.omg.CORBA.TypeCode typeCode_;

    public static org.omg.CORBA.TypeCode
    type()
    {
        if(typeCode_ == null)
        {
            org.omg.CORBA.ORB orb = org.omg.CORBA.ORB.init();
            org.omg.CORBA.StructMember[] members = new org.omg.CORBA.StructMember[15];

            members[0] = new org.omg.CORBA.StructMember();
            members[0].name = "label";
            members[0].type = orb.get_primitive_tc(org.omg.CORBA.TCKind.tk_long);

            members[1] = new org.omg.CORBA.StructMember();
            members[1].name = "n";
            members[1].type = orb.get_primitive_tc(org.omg.CORBA.TCKind.tk_long);

            members[2] = new org.omg.CORBA.StructMember();
            members[2].name = "nbound";
            members[2].type = orb.get_primitive_tc(org.omg.CORBA.TCKind.tk_long);

            members[3] = new org.omg.CORBA.StructMember();
            members[3].name = "nsegment";
            members[3].type = orb.get_primitive_tc(org.omg.CORBA.TCKind.tk_long);

            members[4] = new org.omg.CORBA.StructMember();
            members[4].name = "npoint";
            members[4].type = orb.get_primitive_tc(org.omg.CORBA.TCKind.tk_long);

            members[5] = new org.omg.CORBA.StructMember();
            members[5].name = "ncnet";
            members[5].type = orb.get_primitive_tc(org.omg.CORBA.TCKind.tk_long);

            members[6] = new org.omg.CORBA.StructMember();
            members[6].name = "area";
            members[6].type = orb.get_primitive_tc(org.omg.CORBA.TCKind.tk_long);

            members[7] = new org.omg.CORBA.StructMember();
            members[7].name = "width";
            members[7].type = orb.get_primitive_tc(org.omg.CORBA.TCKind.tk_long);

            members[8] = new org.omg.CORBA.StructMember();
            members[8].name = "type";
            members[8].type = orb.get_primitive_tc(org.omg.CORBA.TCKind.tk_long);

            members[9] = new org.omg.CORBA.StructMember();
            members[9].name = "intensity";
            members[9].type = orb.get_primitive_tc(org.omg.CORBA.TCKind.tk_float);

            members[10] = new org.omg.CORBA.StructMember();
            members[10].name = "std_deviation";
            members[10].type = orb.get_primitive_tc(org.omg.CORBA.TCKind.tk_float);

            members[11] = new org.omg.CORBA.StructMember();
            members[11].name = "bound";
            members[11].type = CBREP_BoundSeqHelper.type();

            members[12] = new org.omg.CORBA.StructMember();
            members[12].name = "segment";
            members[12].type = CBREP_SegmentSeqHelper.type();

            members[13] = new org.omg.CORBA.StructMember();
            members[13].name = "point";
            members[13].type = CBREP_PointSeqHelper.type();

            members[14] = new org.omg.CORBA.StructMember();
            members[14].name = "cnet";
            members[14].type = CBREP_CnetSeqHelper.type();

            typeCode_ = orb.create_struct_tc(id(), "CBREP_Region", members);
        }

        return typeCode_;
    }

    public static String
    id()
    {
        return "IDL:CBREP_Region:1.0";
    }

    public static CBREP_Region
    read(org.omg.CORBA.portable.InputStream in)
    {
        CBREP_Region _ob_v = new CBREP_Region();
        _ob_v.label = in.read_long();
        _ob_v.n = in.read_long();
        _ob_v.nbound = in.read_long();
        _ob_v.nsegment = in.read_long();
        _ob_v.npoint = in.read_long();
        _ob_v.ncnet = in.read_long();
        _ob_v.area = in.read_long();
        _ob_v.width = in.read_long();
        _ob_v.type = in.read_long();
        _ob_v.intensity = in.read_float();
        _ob_v.std_deviation = in.read_float();
        _ob_v.bound = CBREP_BoundSeqHelper.read(in);
        _ob_v.segment = CBREP_SegmentSeqHelper.read(in);
        _ob_v.point = CBREP_PointSeqHelper.read(in);
        _ob_v.cnet = CBREP_CnetSeqHelper.read(in);
        return _ob_v;
    }

    public static void
    write(org.omg.CORBA.portable.OutputStream out, CBREP_Region val)
    {
        out.write_long(val.label);
        out.write_long(val.n);
        out.write_long(val.nbound);
        out.write_long(val.nsegment);
        out.write_long(val.npoint);
        out.write_long(val.ncnet);
        out.write_long(val.area);
        out.write_long(val.width);
        out.write_long(val.type);
        out.write_float(val.intensity);
        out.write_float(val.std_deviation);
        CBREP_BoundSeqHelper.write(out, val.bound);
        CBREP_SegmentSeqHelper.write(out, val.segment);
        CBREP_PointSeqHelper.write(out, val.point);
        CBREP_CnetSeqHelper.write(out, val.cnet);
    }
}
