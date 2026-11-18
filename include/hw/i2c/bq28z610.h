/*
 * TI bq28z610 battery fuel gauge (SBS + MAC mailbox), I2C address 0x55 on
 * i2c7 / 0x11d04000 of the Xiaomi xaga (MT6895).
 *
 * The battery condition is a modelled physical input with a documented
 * default: 3.85 V, 55 % SOC, 25 C, 0.5 A discharge, 5000 mAh design/full
 * capacity, 10 cycles.  All of it is device properties.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_I2C_BQ28Z610_H
#define HW_I2C_BQ28Z610_H

#include "hw/i2c/i2c.h"
#include "qom/object.h"

#define TYPE_BQ28Z610 "bq28z610"
OBJECT_DECLARE_SIMPLE_TYPE(Bq28z610State, BQ28Z610)

#define BQ28Z610_MAC_BUF 40

struct Bq28z610State {
    I2CSlave parent_obj;

    /* Modelled battery state (physical input). */
    uint16_t voltage_mv;
    int32_t current_ma;
    uint16_t temperature_dk;
    uint16_t soc;
    uint16_t soh;
    uint16_t remaining_mah;
    uint16_t full_mah;
    uint16_t design_mah;
    uint16_t cycles;
    uint16_t chg_voltage_mv;
    uint16_t chg_current_ma;
    uint16_t cell_mv[2];

    /* Register/transaction state. */
    uint8_t ptr;
    bool ptr_valid;
    uint16_t control;
    uint8_t rx[2];
    unsigned rx_len;
    unsigned rx_pos;
    uint8_t mac_cmd_lo;
    uint8_t mac_buf[BQ28Z610_MAC_BUF];
    unsigned mac_len;
    bool trace;
};

#endif /* HW_I2C_BQ28Z610_H */
