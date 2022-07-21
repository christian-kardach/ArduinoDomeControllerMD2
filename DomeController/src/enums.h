#pragma once

enum
{
    BTN_NONE,
    BTN_A_CW,
    BTN_A_CCW
};

enum AzimuthEvent
{
    EVT_NONE,
    EVT_GOTO,
    EVT_HOME,
    EVT_ABORT
};

enum AzimuthStatus
{
    AZ_IDLE,
    AZ_GOING,
    AZ_GOING_SLOW,
    AZ_HOMING,
    AZ_ERROR,
};

// MaxDome II azimuth status
enum MDAzimuthStatus
{
    AS_IDLE = 1,
    AS_MOVING_CW,
    AS_MOVING_CCW,
    AS_IDLE2,
    AS_ERROR
};

// MaxDome II shutter status
enum ShutterStatus
{
    SS_CLOSED = 0,
    SS_OPENING,
    SS_OPEN,
    SS_CLOSING,
    SS_ABORTED,
    SS_ERROR
};

// States
enum State {
    ST_HOMED,
    ST_NOTHOMED,
    ST_CW,
    ST_CWING,
    ST_CCW,
    ST_CCWING,
    ST_ABORTED,
    ST_ERROR,
};


enum Action {
    DO_NONE,
    DO_CW,
    DO_CCW,
    DO_ABORT,
};
