from dataclasses import dataclass
from cyclonedds.idl import IdlStruct

@dataclass
class ChatData(IdlStruct, typename="ChatData"):
    text_data: str
