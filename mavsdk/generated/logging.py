# -*- coding: utf-8 -*-
from .._base import AsyncBase
from ..generated import logging_pb2, logging_pb2_grpc
from enum import Enum


class LoggingRaw:
    """ Generated by dcsdkgen """

    

    def __init__(
            self,
            first_message_offset,
            data):
        """ Initializes the LoggingRaw object """
        self.first_message_offset = first_message_offset
        self.data = data

    def __equals__(self, to_compare):
        """ Checks if two LoggingRaw are the same """
        try:
            # Try to compare - this likely fails when it is compared to a non
            # LoggingRaw object
            return \
                (self.first_message_offset == to_compare.first_message_offset) and \
                (self.data == to_compare.data)

        except AttributeError:
            return False

    def __str__(self):
        """ LoggingRaw in string representation """
        struct_repr = ", ".join([
                "first_message_offset: " + str(self.first_message_offset),
                "data: " + str(self.data)
                ])

        return f"LoggingRaw: [{struct_repr}]"

    @staticmethod
    def translate_from_rpc(rpcLoggingRaw):
        """ Translates a gRPC struct to the SDK equivalent """
        return LoggingRaw(
                
                rpcLoggingRaw.first_message_offset,
                
                
                rpcLoggingRaw.data
                )

    def translate_to_rpc(self, rpcLoggingRaw):
        """ Translates this SDK object into its gRPC equivalent """

        
        
            
        rpcLoggingRaw.first_message_offset = self.first_message_offset
            
        
        
        
            
        rpcLoggingRaw.data = self.data
            
        
        


class LoggingResult:
    """ Generated by dcsdkgen """

    
    
    class Result(Enum):
        """ Generated by dcsdkgen """

        
        SUCCESS = 0
        NO_SYSTEM = 1
        CONNECTION_ERROR = 2
        BUSY = 3
        COMMAND_DENIED = 4
        TIMEOUT = 5
        UNKNOWN = 6

        def translate_to_rpc(self, rpcResult):
            rpcResult = {
                    0: logging_pb2.LoggingResult.SUCCESS,
                    1: logging_pb2.LoggingResult.NO_SYSTEM,
                    2: logging_pb2.LoggingResult.CONNECTION_ERROR,
                    3: logging_pb2.LoggingResult.BUSY,
                    4: logging_pb2.LoggingResult.COMMAND_DENIED,
                    5: logging_pb2.LoggingResult.TIMEOUT,
                    6: logging_pb2.LoggingResult.UNKNOWN
                }.get(self.value, None)

        @staticmethod
        def translate_from_rpc(rpc_enum_value):
            """ Parses a gRPC response """
            return {
                    0: LoggingResult.Result.SUCCESS,
                    1: LoggingResult.Result.NO_SYSTEM,
                    2: LoggingResult.Result.CONNECTION_ERROR,
                    3: LoggingResult.Result.BUSY,
                    4: LoggingResult.Result.COMMAND_DENIED,
                    5: LoggingResult.Result.TIMEOUT,
                    6: LoggingResult.Result.UNKNOWN,
                }.get(rpc_enum_value, None)

        def __str__(self):
            return self.name
    

    def __init__(
            self,
            result,
            result_str):
        """ Initializes the LoggingResult object """
        self.result = result
        self.result_str = result_str

    def __equals__(self, to_compare):
        """ Checks if two LoggingResult are the same """
        try:
            # Try to compare - this likely fails when it is compared to a non
            # LoggingResult object
            return \
                (self.result == to_compare.result) and \
                (self.result_str == to_compare.result_str)

        except AttributeError:
            return False

    def __str__(self):
        """ LoggingResult in string representation """
        struct_repr = ", ".join([
                "result: " + str(self.result),
                "result_str: " + str(self.result_str)
                ])

        return f"LoggingResult: [{struct_repr}]"

    @staticmethod
    def translate_from_rpc(rpcLoggingResult):
        """ Translates a gRPC struct to the SDK equivalent """
        return LoggingResult(
                
                LoggingResult.Result.translate_from_rpc(rpcLoggingResult.result),
                
                
                rpcLoggingResult.result_str
                )

    def translate_to_rpc(self, rpcLoggingResult):
        """ Translates this SDK object into its gRPC equivalent """

        
        
            
        self.result.translate_to_rpc(rpcLoggingResult.result)
            
        
        
        
            
        rpcLoggingResult.result_str = self.result_str
            
        
        



class LoggingError(Exception):
    """ Raised when a LoggingResult is a fail code """

    def __init__(self, result, origin, *params):
        self._result = result
        self._origin = origin
        self._params = params

    def __str__(self):
        return f"{self._result.result}: '{self._result.result_str}'; origin: {self._origin}; params: {self._params}"


class Logging(AsyncBase):
    """ Generated by dcsdkgen - MAVSDK Logging API """

    # Plugin name
    name = "Logging"

    def _setup_stub(self, channel):
        """ Setups the api stub """
        self._stub = logging_pb2_grpc.LoggingServiceStub(channel)

    
    def _extract_result(self, response):
        """ Returns the response status and description """
        return LoggingResult.translate_from_rpc(response.logging_result)
    

    async def start_logging(self):
        """ Generated by dcsdkgen

        :returns: Tuple[Success, Response]
        """

        request = logging_pb2.StartLoggingRequest()
        response = await self._stub.StartLogging(request)

        
        result = self._extract_result(response)

        if result.result is not LoggingResult.Result.SUCCESS:
            raise LoggingError(result, "start_logging()")
        

    async def stop_logging(self):
        """ Generated by dcsdkgen

        :returns: Tuple[Success, Response]
        """

        request = logging_pb2.StopLoggingRequest()
        response = await self._stub.StopLogging(request)

        
        result = self._extract_result(response)

        if result.result is not LoggingResult.Result.SUCCESS:
            raise LoggingError(result, "stop_logging()")
        

    async def logging_raw(self):
        """ Generated by dcsdkgen """
        request = logging_pb2.SubscribeLoggingRawRequest()
        self.logging_raw_stream = self._stub.SubscribeLoggingRaw(request)

        try:
            async for response in self.logging_raw_stream:
                

            
                yield LoggingRaw.translate_from_rpc(response.logging_raw)
        finally:
            self.logging_raw_stream.cancel()