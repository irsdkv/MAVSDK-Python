# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
import grpc

from . import gimbal_pb2 as gimbal__pb2


class GimbalServiceStub(object):
  # missing associated documentation comment in .proto file
  pass

  def __init__(self, channel):
    """Constructor.

    Args:
      channel: A grpc.Channel.
    """
    self.SetPitchAndYaw = channel.unary_unary(
        '/mavsdk.rpc.gimbal.GimbalService/SetPitchAndYaw',
        request_serializer=gimbal__pb2.SetPitchAndYawRequest.SerializeToString,
        response_deserializer=gimbal__pb2.SetPitchAndYawResponse.FromString,
        )
    self.SetMode = channel.unary_unary(
        '/mavsdk.rpc.gimbal.GimbalService/SetMode',
        request_serializer=gimbal__pb2.SetModeRequest.SerializeToString,
        response_deserializer=gimbal__pb2.SetModeResponse.FromString,
        )


class GimbalServiceServicer(object):
  # missing associated documentation comment in .proto file
  pass

  def SetPitchAndYaw(self, request, context):
    # missing associated documentation comment in .proto file
    pass
    context.set_code(grpc.StatusCode.UNIMPLEMENTED)
    context.set_details('Method not implemented!')
    raise NotImplementedError('Method not implemented!')

  def SetMode(self, request, context):
    # missing associated documentation comment in .proto file
    pass
    context.set_code(grpc.StatusCode.UNIMPLEMENTED)
    context.set_details('Method not implemented!')
    raise NotImplementedError('Method not implemented!')


def add_GimbalServiceServicer_to_server(servicer, server):
  rpc_method_handlers = {
      'SetPitchAndYaw': grpc.unary_unary_rpc_method_handler(
          servicer.SetPitchAndYaw,
          request_deserializer=gimbal__pb2.SetPitchAndYawRequest.FromString,
          response_serializer=gimbal__pb2.SetPitchAndYawResponse.SerializeToString,
      ),
      'SetMode': grpc.unary_unary_rpc_method_handler(
          servicer.SetMode,
          request_deserializer=gimbal__pb2.SetModeRequest.FromString,
          response_serializer=gimbal__pb2.SetModeResponse.SerializeToString,
      ),
  }
  generic_handler = grpc.method_handlers_generic_handler(
      'mavsdk.rpc.gimbal.GimbalService', rpc_method_handlers)
  server.add_generic_rpc_handlers((generic_handler,))
