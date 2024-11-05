from typing import Iterable
import struct
import math
from sensor_msgs.msg import PointCloud2, PointField
import sys

_DATATYPES = {}
_DATATYPES[PointField.INT8] = ("b", 1)
_DATATYPES[PointField.UINT8] = ("B", 1)
_DATATYPES[PointField.INT16] = ("h", 2)
_DATATYPES[PointField.UINT16] = ("H", 2)
_DATATYPES[PointField.INT32] = ("i", 4)
_DATATYPES[PointField.UINT32] = ("I", 4)
_DATATYPES[PointField.FLOAT32] = ("f", 4)
_DATATYPES[PointField.FLOAT64] = ("d", 8)

def read_points(cloud: PointCloud2, field_names: Iterable | None = None, skip_nans: bool = False, uvs: Iterable = []):
    """
    Read points from a :class:`sensor_msgs.PointCloud2` message.

    :param cloud: The point cloud to read from.
    :type  cloud: class:`sensor_msgs.msg.PointCloud2`
    :param field_names: The names of fields to read. If None, read all fields. [default: None]
    :type  field_names: iterable
    :param skip_nans: If True, then don't return any point with a NaN value.
    :type  skip_nans: bool [default: False]
    :param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    :type  uvs: iterable
    :return: Generator which yields a list of values for each point.
    :rtype:  generator
    """
    assert isinstance(cloud, PointCloud2), "cloud is not a sensor_msgs.msg.PointCloud2"
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = (
        cloud.width,
        cloud.height,
        cloud.point_step,
        cloud.row_step,
        cloud.data,
        math.isnan,
    )
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step


def _get_struct_fmt(is_bigendian: bool, fields: Iterable, field_names: Iterable | None = None):
    """
    Get the struct format string for the given fields.
    
    :param is_bigendian: Whether the data is big-endian.
    :type  is_bigendian: bool
    :param fields: The point fields to read.
    :type  fields: iterable
    :param field_names: The names of fields to read. If None, read all fields. [default: None]
    :type  field_names: iterable
    :return: The struct format string.
    :rtype:  str"""
    fmt = ">" if is_bigendian else "<"

    offset = 0
    for field in (
        f
        for f in sorted(fields, key=lambda f: f.offset)
        if field_names is None or f.name in field_names
    ):
        if offset < field.offset:
            fmt += "x" * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print(
                "Skipping unknown PointField datatype [%d]" % field.datatype,
                file=sys.stderr,
            )
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt