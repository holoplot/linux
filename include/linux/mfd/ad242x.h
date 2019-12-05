/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __LINUX_MFD_AD242X_H
#define __LINUX_MFD_AD242X_H

#define AD242X_CHIP			0x00
#define AD242X_NODEADR			0x01
#define AD242X_NODEADR_MASK		0x0f
#define AD242X_NODEADR_PERI		BIT(5)
#define AD242X_NODEADR_BRCST		BIT(7)

#define AD242X_VENDOR			0x02
#define AD242X_PRODUCT			0x03
#define AD242X_VERSION			0x04

#define AD242X_CAPABILITY		0x05
#define AD242X_CAPABILITY_I2C		BIT(0)

#define AD242X_SWCTL			0x09
#define AD242X_SWCTL_ENSW		BIT(0)
#define AD242X_SWCTL_DIAGMODE		BIT(3)
#define AD242X_SWCTL_MODE(X)		(((X) & 3) << 4)
#define AD242X_SWCTL_MODE_MASK		(3 << 4)
#define AD242X_SWCTL_DISNXT		BIT(6)

#define AD242X_BCDNSLOTS		0x0a
#define AD242X_BCDNSLOTS_MASK		0x3f

#define AD242X_LDNSLOTS			0x0b
#define AD242X_LDNSLOTS_MASK		0x3f
#define AD242X_LDNSLOTS_DNMASKEN	BIT(7)

#define AD242X_LUPSLOTS			0x0c
#define AD242X_LUPSLOTS_MASK		0x3f

#define AD242X_DNSLOTS			0x0d
#define AD242X_DNSLOTS_MASK		0x3f

#define AD242X_UPSLOTS			0x0e
#define AD242X_UPSLOTS_MASK		0x3f

#define AD242X_RESPCYCS			0x0f

#define AD242X_SLOTFMT			0x10
#define AD242X_SLOTFMT_DNSIZE(X)	((((X) - 8) >> 2) & 7)
#define AD242X_SLOTFMT_DNFMT		BIT(3)
#define AD242X_SLOTFMT_UPSIZE(X)	(((((X) - 8) >> 2) & 7) << 4)
#define AD242X_SLOTFMT_UPFMT		BIT(7)

#define AD242X_DATCTL			0x11
#define AD242X_DATCTL_DNS		BIT(0)
#define AD242X_DATCTL_UPS		BIT(1)
#define AD242X_DATCTL_ENDSNIFF		BIT(5)
#define AD242X_DATCTL_STANDBY		BIT(7)

#define AD242X_CONTROL			0x12
#define AD242X_CONTROL_NEWSTRCT		BIT(0)
#define AD242X_CONTROL_ENDDSC		BIT(1)
#define AD242X_CONTROL_SOFTRST		BIT(2)
#define AD242X_CONTROL_SWBYP		BIT(3)
#define AD242X_CONTROL_XCVRBINV		BIT(4)
#define AD242X_CONTROL_MSTR		BIT(7)

#define AD242X_DISCVRY			0x13

#define AD242X_SWSTAT			0x14
#define AD242X_SWSTAT_FIN		BIT(0)
#define AD242X_SWSTAT_FAULT		BIT(1)
#define AD242X_SWSTAT_FAULTCODE(X)	(((X) & 0x7) >> 4)
#define AD242X_SWSTAT_FAULT_NLOC	BIT(7)

#define AD242X_INTSTAT			0x15
#define AD242X_INTSTAT_IRQ		BIT(0)

#define AD242X_INTSRC			0x16
#define AD242X_INTSRC_INODE		0x0f
#define AD242X_INTSRC_SLVINT		BIT(6)
#define AD242X_INTSRC_MSTINT		BIT(7)

#define AD242X_INTTYPE			0x17

#define AD242X_INTTYPE_IO0PND		16
#define AD242X_INTTYPE_IO7PND		23
#define AD242X_INTTYPE_DSCDONE		24
#define AD242X_INTTYPE_MSTR_RUNNING	255

#define AD242X_INTPND0			0x18
#define AD242X_INTPDN0_HDCNTERR		BIT(0)
#define AD242X_INTPDN0_DDERR		BIT(1)
#define AD242X_INTPDN0_CRCERR		BIT(2)
#define AD242X_INTPDN0_DPERR		BIT(3)
#define AD242X_INTPDN0_PWRERR		BIT(4)
#define AD242X_INTPDN0_BECOVF		BIT(5)
#define AD242X_INTPDN0_SRFERR		BIT(6)
#define AD242X_INTPDN0_SRFCRCERR	BIT(7)

#define AD242X_INTPND1			0x19

#define AD242X_INTPND2			0x1a
#define AD242X_INTPND2_DSCDONE		BIT(0)
#define AD242X_INTPND2_I2CERR		BIT(1)
#define AD242X_INTPND2_ICRCERR		BIT(2)
#define AD242X_INTPND2_SLVIRQ		BIT(3)

#define AD242X_INTMSK0			0x1b
#define AD242X_INTMSK0_HCEIEN		BIT(0)
#define AD242X_INTMSK0_DDEIEN		BIT(1)
#define AD242X_INTMSK0_CRCEIEN		BIT(2)
#define AD242X_INTMSK0_DPEIEN		BIT(3)
#define AD242X_INTMSK0_PWREIEN		BIT(4)
#define AD242X_INTMSK0_BECIEN		BIT(5)
#define AD242X_INTMSK0_SRFEIEN		BIT(6)
#define AD242X_INTMSK0_SRFCRCEIEN	BIT(7)

#define AD242X_INTMSK1			0x1c
#define AD242X_INTMSK1_IOIRQEN(X)	BIT(X)

#define AD242X_INTMSK2			0x1d
#define AD242X_INTMSK2_DSCDIEN		BIT(0)
#define AD242X_INTMSK2_I2CEIEN		BIT(1)
#define AD242X_INTMSK2_ICRCEIEN		BIT(2)
#define AD242X_INTMSK2_SLVIRQEN		BIT(3)

#define AD242X_BECCTL			0x1e
#define AD242X_BECCTL_ENHDCNT		BIT(0)
#define AD242X_BECCTL_ENDD		BIT(1)
#define AD242X_BECCTL_ENCRC		BIT(2)
#define AD242X_BECCTL_ENDP		BIT(3)
#define AD242X_BECCTL_ENICRC		BIT(4)
#define AD242X_BECCTL_THRESHLD(X)	((X) >> 5)

#define AD242X_BECNT			0x1f

#define AD242X_TESTMODE			0x20
#define AD242X_TESTMODE_PRBSUP		BIT(0)
#define AD242X_TESTMODE_PRBSDN		BIT(1)
#define AD242X_TESTMODE_PRBSN2N		BIT(2)
#define AD242X_TESTMODE_RXDPTH(X)	((X) >> 4)

#define AD242X_ERRCNT0			0x21
#define AD242X_ERRCNT1			0x22
#define AD242X_ERRCNT2			0x23
#define AD242X_ERRCNT3			0x24

#define AD242X_NODE			0x29
#define AD242X_NODE_MASK		0xf
#define AD242X_NODE_DISCVD		BIT(5)
#define AD242X_NODE_NLAST		BIT(6)
#define AD242X_NODE_LAST		BIT(7)

#define AD242X_DISCSTAT			0x2b
#define AD242X_DISCSTAT_DNODE(X)	((X) & 0xf)
#define AD242X_DISCSTAT_DSCACT		BIT(7)

#define AD242X_TXACTL			0x2e
#define AD242X_TXACTL_LEVEL_HIGH	0
#define AD242X_TXACTL_LEVEL_MEDIUM	2
#define AD242X_TXACTL_LEVEL_LOW		3

#define AD242X_TXBCTL			0x30
#define AD242X_TXBCTL_LEVEL_HIGH	0
#define AD242X_TXBCTL_LEVEL_MEDIUM	2
#define AD242X_TXBCTL_LEVEL_LOW		3

#define AD242X_LINTTYPE			0x3e

#define AD242X_I2CCFG			0x3f
#define AD242X_I2CCFG_DATARATE		BIT(0)
#define AD242X_I2CCFG_EACK		BIT(1)
#define AD242X_I2CCFG_FRAMERATE		BIT(2)

#define AD242X_PLLCTL			0x40
#define AD242X_PLLCTL_SSFREQ(X)		((X) & 3)
#define AD242X_PLLCTL_SSDEPTH		BIT(2)
#define AD242X_PLLCTL_SSMODE_AB		(1 << 6)
#define AD242X_PLLCTL_SSMODE_AB_I2S	(2 << 6)

#define AD242X_I2SGCTL			0x41
#define AD242X_I2SGCTL_TDMMODE(X)	((X) & 3)
#define AD242X_I2SGCTL_RXONDTX1		BIT(3)
#define AD242X_I2SGCTL_TDMSS		BIT(4)
#define AD242X_I2SGCTL_ALT		BIT(5)
#define AD242X_I2SGCTL_EARLY		BIT(6)
#define AD242X_I2SGCTL_INV		BIT(7)

#define AD242X_I2SCTL			0x42
#define AD242X_I2SCTL_TX0EN		BIT(0)
#define AD242X_I2SCTL_TX1EN		BIT(1)
#define AD242X_I2SCTL_TX2PINTL		BIT(2)
#define AD242X_I2SCTL_TXBCLKINV		BIT(3)
#define AD242X_I2SCTL_RX0EN		BIT(4)
#define AD242X_I2SCTL_RX1EN		BIT(5)
#define AD242X_I2SCTL_RX2PINTL		BIT(6)
#define AD242X_I2SCTL_RXBCLKINV		BIT(7)

#define AD242X_I2SRATE			0x43
#define AD242X_I2SRATE_I2SRATE(X)	((X) & 3)
#define AD242X_I2SRATE_BCLKRATE(X)	(((X) << 3) & 3)
#define AD242X_I2SRATE_REDUCE		BIT(6)
#define AD242X_I2SRATE_SHARE		BIT(7)

#define AD242X_I2STXOFFSET		0x44
#define AD242X_I2STXOFFSET_VAR(X)	((X) & 0x3f)
#define AD242X_I2STXOFFSET_TSAFTER	BIT(6)
#define AD242X_I2STXOFFSET_TSBEFORE	BIT(7)

#define AD242X_2SRXOFFSET		0x45
#define AD242X_I2SRXOFFSET_VAR(X)	((X) & 0x3f)

#define AD242X_SYNCOFFSET		0x46

#define AD242X_PDMCTL			0x47
#define AD242X_PDMCTL_PDM0EN		BIT(0)
#define AD242X_PDMCTL_PDM0SLOTS		BIT(1)
#define AD242X_PDMCTL_PDM1EN		BIT(2)
#define AD242X_PDMCTL_PDM1SLOTS		BIT(3)
#define AD242X_PDMCTL_HPFEN		BIT(4)
#define AD242X_PDMCTL_PDMRATE(X)	(((X) & 3) << 5)

#define AD242X_ERRMGMT			0x48
#define AD242X_ERRMGMT_ERRLSB		BIT(0)
#define AD242X_ERRMGMT_ERRSIG		BIT(1)
#define AD242X_ERRMGMT_ERRSLOT		BIT(2)

#define AD242X_GPIODAT			0x4a
#define AD242X_GPIODAT_SET		0x4b
#define AD242X_GPIODAT_CLR		0x4c
#define AD242X_GPIOOEN			0x4d
#define AD242X_GPIOIEN			0x4e
#define AD242X_GPIODAT_IN		0x4f
#define AD242X_PINTEN			0x50
#define AD242X_PINTINV			0x51

#define AD242X_PINCFG			0x52
#define AD242X_PINCFG_DRVSTR		BIT(0)
#define AD242X_PINCFG_IRQINV		BIT(4)
#define AD242X_PINCFG_IRQTS		BIT(5)

#define AD242X_I2STEST			0x53
#define AD242X_I2STEST_PATTRN2TX	BIT(0)
#define AD242X_I2STEST_LOOPBK2TX	BIT(1)
#define AD242X_I2STEST_RX2LOOPBK	BIT(2)
#define AD242X_I2STEST_SELRX1		BIT(3)
#define AD242X_I2STEST_BUSLOOPBK	BIT(4)

#define AD242X_RAISE			0x54

#define AD242X_GENERR			0x55
#define AD242X_GENERR_GENHCERR		BIT(0)
#define AD242X_GENERR_GENDDERR		BIT(1)
#define AD242X_GENERR_GENCRCERR		BIT(2)
#define AD242X_GENERR_GENDPERR		BIT(3)
#define AD242X_GENERR_GENICRCERR	BIT(4)

#define AD242X_I2SRRATE			0x56
#define AD242X_I2SRRATE_RRDIV(X)	((X) & 0x3f)
#define AD242X_I2SRRATE_RBUS		BIT(7)

#define AD242X_I2SRRCTL			0x57
#define AD242X_I2SRRCTL_ENVLSB		BIT(0)
#define AD242X_I2SRRCTL_ENXBIT		BIT(1)
#define AD242X_I2SRRCTL_ENSTRB		BIT(4)
#define AD242X_I2SRRCTL_STRBDIR		BIT(5)

#define AD242X_I2SRRSOFFS		0x58

#define AD242X_CLK1CFG			0x59
#define AD242X_CLK2CFG			0x5a
#define AD242X_CLKCFG_DIV(X)		((X) & 0xf)
#define AD242X_CLKCFG_DIVMSK		0xf
#define AD242X_CLKCFG_PDIV32		BIT(5)
#define AD242X_CLKCFG_INV		BIT(6)
#define AD242X_CLKCFG_EN		BIT(7)

#define AD242X_BMMCFG			0x5b
#define AD242X_BMMCFG_BMMEN		BIT(0)
#define AD242X_BMMCFG_BMMRXEN		BIT(1)
#define AD242X_BMMCFG_BMMNDSC		BIT(2)

#define AD242X_PDMCTL2			0x5d
#define AD242X_PDMCTL2_DEST_A2B		0
#define AD242X_PDMCTL2_DEST_DTX		1
#define AD242X_PDMCTL2_DEST_A2B_DTX	2

#define AD242X_UPMASK(X)		(0x60 + ((X) & 3))

#define AD242X_UPOFFSET			0x64
#define AD242X_UPOFFSET_VAL(X)		((X) & 0x1f)

#define AD242X_DNMASK(X)		(0x65 + ((X) & 3))

#define AD242X_DNOFFSET			0x69
#define AD242X_DNOFFSET_VAL(X)		((X) & 0x1f)

#define AD242X_CHIPID(X)		((X) + 0x6a)

#define AD242X_GPIODEN			0x80
#define AD242X_GPIOD_MSK(X)		((X) + 0x81)

#define AD242X_GPIODDAT			0x89
#define AD242X_GPIODINV			0x8a

#define AD242X_MAX_REG			0x9b

static inline bool ad242x_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AD242X_VENDOR:
	case AD242X_PRODUCT:
	case AD242X_VERSION:
	case AD242X_CAPABILITY:
	case AD242X_DATCTL:
	case AD242X_CONTROL:
	case AD242X_SWSTAT:
	case AD242X_INTSTAT:
	case AD242X_INTSRC:
	case AD242X_INTTYPE:
	case AD242X_INTPND0:
	case AD242X_INTPND1:
	case AD242X_INTPND2:
	case AD242X_BECNT:
	case AD242X_ERRCNT0:
	case AD242X_ERRCNT1:
	case AD242X_ERRCNT2:
	case AD242X_ERRCNT3:
	case AD242X_NODE:
	case AD242X_DISCSTAT:
	case AD242X_LINTTYPE:
	case AD242X_GPIODAT:
	case AD242X_GPIODAT_IN:
		return true;
	default:
		return false;
	}
}

static inline bool ad242x_is_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AD242X_DATCTL:
	case AD242X_CONTROL:
	/* Write-to-clean registers */
	case AD242X_INTPND0:
	case AD242X_INTPND1:
	case AD242X_INTPND2:
	case AD242X_BECNT:
		return true;
	default:
		return !ad242x_is_volatile_reg(dev, reg);
	}
}

#define AD242X_MASTER_ID 0xff

struct ad242x_master;

struct ad242x_i2c_bus {
	struct i2c_client	*client;
	struct mutex		mutex;
};

struct ad242x_node {
	struct device		*dev;
	struct regmap		*regmap;
	struct ad242x_master	*master;
	unsigned int		tdm_mode;
	unsigned int		tdm_slot_size;
	uint8_t			id;
	uint8_t			caps;
};

struct ad242x_slot_config {
	unsigned int dn_rx_slots;
	unsigned int dn_n_tx_slots;
	unsigned int dn_n_forward_slots;
	unsigned int up_rx_slots;
	unsigned int up_n_tx_slots;
	unsigned int up_n_forward_slots;
};

int ad242x_read_slot_config(struct device *dev,
			    struct device_node *np,
			    struct ad242x_slot_config *config);

static inline bool ad242x_node_is_master(struct ad242x_node *node)
{
	return node->id == AD242X_MASTER_ID;
}

int ad242x_node_probe(struct ad242x_node *node);
int ad242x_node_add_mfd_cells(struct device *dev);

struct ad242x_node *ad242x_master_get_node(struct ad242x_master *master);
struct ad242x_i2c_bus *ad242x_master_get_bus(struct ad242x_master *master);
const char *ad242x_master_get_clk_name(struct ad242x_master *master);
unsigned int ad242x_master_get_clk_rate(struct ad242x_master *master);
u8 ad242x_node_inttype(struct ad242x_node *node);

int ad242x_slave_read(struct ad242x_i2c_bus *bus,
		      struct regmap *master_regmap,
		      uint8_t node_id, uint8_t reg, unsigned int *val);
int ad242x_slave_write(struct ad242x_i2c_bus *bus,
		       struct regmap *master_regmap,
		       uint8_t node_id, uint8_t reg, unsigned int val);

#endif
