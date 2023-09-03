from enum import IntEnum

class DayNightEnum(IntEnum):
    Unknown = 0
    Day = 1
    Night = 2

class TrackingStateEnum(IntEnum):
    ProvisionaryTarget = 1
    ActiveTarget = 2
    LostTarget = 3