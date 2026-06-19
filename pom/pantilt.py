from scservo_sdk import *

STEP = 3

PAN_MIN = 0
PAN_MAX = 1023

TILT_MIN = 0
TILT_MAX = 1023

pan_pos = 511
tilt_pos = 511


def scs_write_pos(
    scs_packet,
    portHandler,
    servo_id,
    pos
):

    pos = SCS_TOSCS(pos, 0)

    pos_L = SCS_LOBYTE(pos)
    pos_H = SCS_HIBYTE(pos)

    scs_packet.writeTxRx(
        portHandler,
        servo_id,
        42,
        2,
        [pos_H, pos_L]
    )


def update_pantilt(
    adc,
    sw,
    scs_packet,
    portHandler,
    PAN_ID,
    TILT_ID
):

    global pan_pos
    global tilt_pos

    # =========================
    # SW OFF
    # =========================
    if sw != 1:
        return

    # =========================
    # IND INPUT
    # =========================
    ind0 = adc[14]   # IND0
    ind1 = adc[15]   # IND1

    # =========================
    # CENTER 기준값
    # =========================
    center = 2000
    deadzone = 150

    # =========================
    # PAN (IND0)
    # =========================
    if ind0 > center + deadzone:
        pan_pos += STEP
    elif ind0 < center - deadzone:
        pan_pos -= STEP

    # =========================
    # TILT (IND1)
    # =========================
    if ind1 > center + deadzone:
        tilt_pos += STEP
    elif ind1 < center - deadzone:
        tilt_pos -= STEP

    # =========================
    # LIMIT
    # =========================
    if pan_pos > PAN_MAX:
        pan_pos = PAN_MAX
    elif pan_pos < PAN_MIN:
        pan_pos = PAN_MIN

    if tilt_pos > TILT_MAX:
        tilt_pos = TILT_MAX
    elif tilt_pos < TILT_MIN:
        tilt_pos = TILT_MIN

    # =========================
    # WRITE
    # =========================
    scs_write_pos(scs_packet, portHandler, PAN_ID, pan_pos)
    scs_write_pos(scs_packet, portHandler, TILT_ID, tilt_pos)