/* Emulation of DPS310 temperature sensor for openbmc/qemu
 *
 * Written by Monthero Ronald <rhmcruiser@gmail.com>
 * Inspired by Andrew Jeffery and reference of tmp321 
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 or
 * (at your option) version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "hw/hw.h"
#include "hw/i2c/i2c.h"
#include "qapi/error.h"
#include "qapi/visitor.h"

typedef struct DeviceInfo {
    int model;
    const char *name;
} DeviceInfo;

typedef struct DPS310State {
    /* < private > */
    I2CSlave i2c;
    /* < public > */

    int16_t temperature[4];
    int16_t pressure;  

    uint8_t status;
    uint8_t config[2];
    uint8_t rate;

    uint8_t len;
    uint8_t buf[2];
    uint8_t pointer;

   /* For now dps310.c is temp only 
      To add pressure sensor later 
   */
} DPS310State;

typedef struct DPS310Class {
    I2CSlaveClass parent_class;

    DeviceInfo *dev;
} DPS310Class;

#define TYPE_DPS310 "dps310-generic"
#define DPS310(obj) OBJECT_CHECK(DPS310State, (obj), TYPE_DPS310)

#define DPS310_CLASS(class) \
    OBJECT_CLASS_CHECK(DPS310Class, (class), TYPE_DPS310)
#define DPS310_GET_CLASS(obj) \
    OBJECT_GET_CLASS(DPS310Class, (obj), TYPE_DPS310)


/* DPS310 registers */
#define   DPS310_MANUFACTURER_ID_REG  0x0D
#define DPS310_DEVICE_ID    0x10   /* Value on reset at address DPS310_MANUFACTURER_ID_REG 0x0D */

static const DeviceInfo device = { DPS310_DEVICE_ID, "dps310"};

/* STATUS REGISTER - Sensor Operating Mode and Status */

#define DPS310_MEAS_CFG               0x08
#define   DPS310_COEF_RDY               (1 << 7)
#define   DPS310_SENSOR_RDY             (1 << 6)
#define   DPS310_TMP_RDY                (1 << 5)
#define   DPS310_PRS_RDY                (1 << 4)
#define   DPS310_MEAS_CRTL_standby       0x00
#define   DPS310_MEAS_CRTL_pressure     (1 << 1)
#define   DPS310_MEAS_CRTL_temp         (1 << 2)

/*  
 * MEAS_CTRL  bits used are 2, 1, 0  rw Set measurement mode and type:
 * Standby Mode
 * 000 - Idle / Stop background measurement
 * Command Mode
 * 001 - Pressure measurement
 * 010 - Temperature measurement
 * 011 - na.
 * 100 - na.
 * Background Mode
 * 101 - Continous pressure measurement
 * 110 - Continous temperature measurement
 * 111 - Continous pressure and temperature measurement
 */
#define   DPS310_MEAS_CRTL_standby       0x00
#define   DPS310_MEAS_CRTL_pressure     (1 << 1)
#define   DPS310_MEAS_CRTL_temp         (1 << 2)
#define   DPS310_MEAS_CRTL_continous_pres (0xFF & 0x05)
#define   DPS310_MEAS_CRTL_continous_temp (0xFF & 0x06)
#define   DPS310_MEAS_CRTL_cont_temp_n_pres (0xFF & 0x07)

/* 
 * Interrupt and FIFO configuration (CFG_REG) register
 *
Field Bits Type Description
INT_HL 7 rw Interupt (on SDO pin) active level:
0 - Active low.
1 - Active high.
INT_FIFO 6 rw Generate interupt when the FIFO is full:
0 - Disable.
1 - Enable.
INT_TMP 5 rw Generate interupt when a temperature measurement is ready:
0 - Disable.
1 - Enable.
INT_PRS 4 rw Generate interupt when a pressure measurement is ready:
0 - Disable.
1 - Enable.
T_SHIFT 3 rw Temperature result bit-shift
0 - no shift.
1 - shift result right in data register.
Note: Must be set to '1' when the oversampling rate is >8 times.
P_SHIFT 2 rw Pressure result bit-shift
0 - no shift.
1 - shift result right in data register.
Note: Must be set to '1' when the oversampling rate is >8 times.
FIFO_EN 1 rw Enable the FIFO:
0 - Disable.
1 - Enable.
SPI_MODE 0 rw Set SPI mode:
0 - 4-wire interface.
1 - 3-wire interface.
*/

#define   DPS310_CFG_REG                0X09    
#define     DPS310_CFG_REG_int_hl         (1 << 7)
#define     DPS310_CFG_REG_int_fifo       (1 << 6)              
#define     DPS310_CFG_REG_int_tmp        (1 << 5)
#define     DPS310_CFG_REG_int_prs        (1 << 4)
#define     DPS310_CFG_REG_t_shift        (1 << 3) 
#define     DPS310_CFG_REG_p_shift        (1 << 2) 
#define     DPS310_CFG_REG_fifo_en        (1 << 1)

#define     DPS310_CFG_REG_spi_mode_4wire  0x0
#define     DPS310_CFG_REG_spi_mode_3wire  0x1

/* Interrupt Status (INT_STS) register

 *Field Bits Type Description
-   7:3 - Reserved.
INT_FIFO_FULL 2 r Status of FIFO interrupt
0 - Interrupt not active
1 - Interrupt active
INT_TMP 1 r Status of temperature measurement interrupt
0 - Interrupt not active
1 - Interrupt active
INT_PRS 0 r Status of pressure measurement interrupt
0 - Interrupt not active
1 - Interrupt active
*/
#define     DPS310_INT_STS                      0x0A
#define       DPS310_INT_STS_int_fifo_full        (1 << 2)
#define       DPS310_INT_STS_int_tmp              (1 << 1)
#define       DPS310_INT_STS_int_prs              (1 << 0)


/* FIFO Status (FIFO_STS) register 
 * Field Bits Type Description
 * - 7:2 - Reserved.
 *   FIFO_FULL bit 1 
 *   0 - The FIFO is not full
 *   1 - The FIFO is full
 *   FIFO_EMPTY bit 0 
 *   0 - The FIFO is not empty
 *   1 - The FIFO is empty
 */
#define     DPS310_FIFO_STS                     0X0B
#define       DPS310_FIFO_STS_fifo_full           (1 << 1)
#define       DPS310_FIFO_STS_fifo_empty          (1 << 0)

/*  Soft Reset and FIFO flush (RESET) register 
 *
 * Field Bits Type Description
 * FIFO_FLUSH bit 7 , type w , FIFO flush
 * 1 - Empty FIFO
 * After reading out all data from the FIFO, write '1' to clear all old
 * data.
 * 6:4 - Reserved.
 * SOFT_RST 3:0,  type w , Write '1001' to generate a soft reset.
 * A soft reset will run though the same sequences as in power-on reset
*/
#define     DPS310_RESET                     0X0C
#define       DPS310_RESET_fifo_flush           (1 << 7)
#define       DPS310_RESET_soft_reset           (0xff & 0x09) 

/*  Coefficient Source register
 *
 * TMP_COEF_SRCE bit 7 type read, Temperature coefficients are based on:
 * 0 - Internal temperature sensor (of ASIC)
 * 1 - External temperature sensor (of pressure sensor MEMS
 * element)
 */
#define    DPS310_TMP_COEF_SRCE         0x28
#define       DPS310_TMP_COEF_SRCE_external        0x01
#define       DPS310_TMP_COEF_SRCE_internal        0x00

#define    DPS310_TMP_MSB0      0x00
#define    DPS310_TMP_MSB1      0x01
#define    DPS310_TMP_LSB0      0x10    
#define    DPS310_TMP_LSB1      0x11

/* Temperature range is -40 to +85 °C */
static const int32_t mins[2] = {-40000, -55000};
static const int32_t maxs[2] =  {85000, 85000};

static void dps310_get_temperature(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    DPS310State *s = DPS310(obj);
    bool temp_range = (s->config[0] & DPS310_MEAS_CFG);
    int offset = temp_range * 64 * 256; /* Revisit if to select mode of temp & pressure */
    int64_t value;
    int tempid;

    if (sscanf(name, "temperature%d", &tempid) != 1) {
        error_setg(errp, "error reading %s: %m", name);
        return;
    }


    if (tempid >= 3 || tempid < 0) {
        error_setg(errp, "error reading %s", name);
        return;
    }

    /* Resolution scaling */
    value = ((s->temperature[tempid] - offset) * 1000 + 128) / 256;

    visit_type_int(v, name, &value, errp);
}

/*
  Units are 0.001 centigrades relative to 0 C.  s->temperature is 8.8
  fixed point, so units are 1/256 centigrades.  A simple ratio will do.
 */

static void dps310_set_temperature(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    DPS310State *s = DPS310(obj);
    Error *local_err = NULL;
    int64_t temp;
    bool ext_range = (s->config[0] & DPS310_MEAS_CFG);
    int offset = ext_range * 64 * 256;
    int tempid;

    visit_type_int(v, name, &temp, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }

    if (temp >= maxs[ext_range] || temp < mins[ext_range]) {
        error_setg(errp, "value %" PRId64 ".%03" PRIu64 " C is out of range",
                   temp / 1000, temp % 1000);
        return;
    }

    if (sscanf(name, "temperature%d", &tempid) != 1) {
        error_setg(errp, "error reading %s: %m", name);
        return;
    }

    if (tempid >= 4 || tempid < 0) {
        error_setg(errp, "error reading %s", name);
        return;
    }

    s->temperature[tempid] = (int16_t) ((temp * 256 - 128) / 1000) + offset;
}

static void dps310_read(DPS310State *s)
{
    DPS310Class *sc = DPS310_GET_CLASS(s);
    s->len = 0;

    switch (s->pointer) {
    case DPS310_MANUFACTURER_ID_REG:
         s->buf[s->len++] = sc->dev->model;
         break;
    case DPS310_CFG_REG:
        s->buf[s->len++] = s->config[0];
        s->buf[s->len++] = s->config[1];
        break;
    case DPS310_TMP_COEF_SRCE:
        s->buf[s->len++] = s->rate;
        break;
    case DPS310_MEAS_CFG:
        s->buf[s->len++] = s->status;
        break;

/* FIXME: check for channel enablement in config registers */
    case DPS310_TMP_MSB1:
        s->buf[s->len++] = (((uint16_t) s->temperature[0]) >> 8);
        s->buf[s->len++] = (((uint16_t) s->temperature[0]) >> 0) & 0xf0;
        break;
    case DPS310_TMP_MSB0:
        s->buf[s->len++] = (((uint16_t) s->temperature[1]) >> 8);
        s->buf[s->len++] = (((uint16_t) s->temperature[1]) >> 0) & 0xf0;
        break;
    case DPS310_TMP_LSB1:
        s->buf[s->len++] = (((uint16_t) s->temperature[0]) >> 0) & 0xf0;
        break;
    case DPS310_TMP_LSB0:
        s->buf[s->len++] = (((uint16_t) s->temperature[1]) >> 0) & 0xf0;
        break;
    }
}

static void dps310_reset(I2CSlave *i2c)
{
    DPS310State *s = DPS310(i2c);
    DPS310Class *sc = DPS310_GET_CLASS(s);

    sc->dev->model = DPS310_DEVICE_ID;
    memset(s->temperature, 0, sizeof(s->temperature));
    s->pointer = 0;

    s->config[0] = 0;
    s->config[1] = 0x1c; /* FIXME: revist to check if needed to be proper init value */


    s->rate = 0x7;       /* 8Hz */  /* FIXME: check init value for rate setting */
    s->status = 0;
}

static void dps310_realize(DeviceState *dev, Error **errp)
{
    DPS310State *s = DPS310(dev);

    dps310_reset(&s->i2c);
}


static void dps310_write(DPS310State *s)
{
    switch (s->pointer) {
    case DPS310_TMP_COEF_SRCE:
        s->rate = s->buf[0];
        break;
    case DPS310_CFG_REG:
        s->config[0] = s->buf[0];
        s->config[1] = s->buf[0];
        break;
    case DPS310_RESET:
        dps310_reset(I2C_SLAVE(s));
        break;
    }
}

static uint8_t dps310_rx(I2CSlave *i2c)
{
    DPS310State  *s = DPS310(i2c);

    if (s->len < 2) {
        return s->buf[s->len++];
    } else {
        return 0xff;
    }
}

static int dps310_tx(I2CSlave *i2c, uint8_t data)
{
    DPS310State *s = DPS310(i2c);

    if (s->len == 0) {
        s->pointer = data;
        s->len++;
    } else if (s->len == 1) {
        s->buf[0] = data;
        dps310_write(s);
    }

    return 0;
}

static int dps310_event(I2CSlave *i2c, enum i2c_event event)
{
    DPS310State *s = DPS310(i2c);

    if (event == I2C_START_RECV) {
        dps310_read(s);
    }

    s->len = 0;
    return 0;
}

static const VMStateDescription vmstate_dps310 = {
    .name = "DPS310",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(len, DPS310State),
        VMSTATE_UINT8_ARRAY(buf, DPS310State, 2),
        VMSTATE_UINT8(pointer, DPS310State),
        VMSTATE_UINT8_ARRAY(config, DPS310State, 2),
        VMSTATE_UINT8(status, DPS310State),
        VMSTATE_UINT8(rate, DPS310State),
        VMSTATE_INT16_ARRAY(temperature, DPS310State, 4),
        VMSTATE_I2C_SLAVE(i2c, DPS310State),
        VMSTATE_END_OF_LIST()
    }
};

static void dps310_initfn(Object *obj)
{
    object_property_add(obj, "temperature0", "int",
                        dps310_get_temperature,
                        dps310_set_temperature, NULL, NULL, NULL);
    object_property_add(obj, "temperature1", "int",
                        dps310_get_temperature,
                        dps310_set_temperature, NULL, NULL, NULL);
    object_property_add(obj, "temperature2", "int",
                        dps310_get_temperature,
                        dps310_set_temperature, NULL, NULL, NULL);
    object_property_add(obj, "temperature3", "int",
                        dps310_get_temperature,
                        dps310_set_temperature, NULL, NULL, NULL);
}

static void dps310_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);
    DPS310Class *sc = DPS310_CLASS(klass);

    dc->realize = dps310_realize;
    k->event = dps310_event;
    k->recv = dps310_rx;
    k->send = dps310_tx;
    dc->vmsd = &vmstate_dps310;
    sc->dev = (DeviceInfo *) data;
}
static const TypeInfo dps310_info = {
    .name          = TYPE_DPS310,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(DPS310State),
    .class_size    = sizeof(DPS310Class),
    .instance_init = dps310_initfn,
    .abstract      = true,
};

static void dps310_register_types(void)
{
    type_register_static(&dps310_info);
        TypeInfo ti = {
            .name       = device.name,
            .parent     = TYPE_DPS310,
            .class_init = dps310_class_init,
            .class_data = (void *) &device,
        };
        type_register(&ti);
    }

type_init(dps310_register_types)
