/*
 * pxdc.c 2017/07/20
 *
 * つかいかた:
 *
 * conf/file.x68k に以下を追加。
 *
 *   defparam opt_pxdc.h PXDC_MASK_ID
 *   device pxdc: scsi
 *   attach pxdc at intio
 *   file   arch/x68k/dev/pxdc.c            pxdc
 *
 * カーネルコンフィグ (conf/GENERIC とか) に以下を追加。
 *
 *   # options	PXDC_MASK_ID=0x01
 *   pxdc*	at intio0 addr 0xeac500
 *   scsibus* at pxdc?
 *
 * PXDC アタッチ時に spc0 の下にぶらさがってるターゲットのうち HDD
 * のみをすべて spc からひっぺがして pxdc 配下に置きます。
 * options PXDC_MASK_ID で (1<<n) ビットを立てると SCSI ID n の HDD
 * を奪わないように指定できます。例えば 0x01 なら SCSI ID 0 は spc
 * 配下のまま、それ以外の SCSI ID の HDD は pxdc 配下になります。
 * SCSI CD、SCSI MO、イニシエータ(ID=7) に対しては意味を持ちません。
 *
 * マウントして読み書きくらいは出来たけど、VM 再起動についてはたぶん
 * まだ未対応。
 */

/*	$NetBSD	*/

/*
 * Copyright (c) 2014 Tetsuya Isaki. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * XM6i virtual disk controller
 */

#include "opt_pxdc.h"

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/buf.h>

#include <machine/cpu.h>
#include <machine/bus.h>

#include <arch/x68k/dev/intiovar.h>

#include <dev/scsipi/scsiconf.h>

#define PXDC_SIG	(0x00)	/* (R-) Signature Reg. */
#define PXDC_TGT	(0x04)	/* (RW) Target Status/Enable */
#define PXDC_DSEL	(0x0c)	/* (RW) Drive Select */
#define PXDC_SCMD	(0x0d)	/* (-W) SCSI Command Start */
#define PXDC_STAT	(0x0e)	/* (R-) Status (for read) */
#define PXDC_RESET	(0x0e)	/* (-W) Reset (for write) */
#define PXDC_SSB	(0x0f)	/* (R-) SCSI Status Byte */
#define PXDC_CDB	(0x10)	/* (RW) CDB Buffer */
#define PXDC_TCNT	(0x20)	/* (R-) Transfer Count */
#define PXDC_ACTR	(0x24)	/* (RW) Array Chain Table Reg. */
#define PXDC_DATA	(0x28)	/* (RW) Data Reg. */

#define PXDC_TGT_ENABLE	(0x01)
#define PXDC_TGT_MASK	(0xf0)
#define PXDC_TGT_NONE	(0x00)
#define PXDC_TGT_HD		(0x10)
#define PXDC_TGT_CD		(0x20)
#define PXDC_TGT_MO		(0x30)
#define PXDC_TGT_INIT	(0xf0)

#define PXDC_SCMD_START6	(0x00)
#define PXDC_SCMD_START10	(0x01)
#define PXDC_SCMD_START12	(0x02)
#define PXDC_SCMD_DMA		(0x80)

#define PXDC_STAT_MASK		(0xf0)
#define PXDC_STAT_DMAERR	(0x80)
#define PXDC_STAT_DSELERR	(0x40)
#define PXDC_STAT_CMDERR	(0x20)
#define PXDC_PHASE_MASK		(0x07)
#define PXDC_PHASE_BUSFREE	(0x00)
#define PXDC_PHASE_DATAIN	(0x01)
#define PXDC_PHASE_DATAOUT	(0x02)
#define PXDC_PHASE_STATUS	(0x03)

#define PXDC_SIGNATURE	0x44433031	/* 'DC01' */

#if defined(PXDC_DEBUG)
#define DPRINTF(fmt...)	printf(fmt)
#else
#define DPRINTF(fmt...)	/**/
#endif

#if !defined(PXDC_MASK_ID)
#define PXDC_MASK_ID	(0x00)
#endif

struct pxdc_softc {
	device_t sc_dev;
	bus_space_tag_t sc_iot;
	bus_space_handle_t sc_ioh;

	int sc_id;			/* my scsi ID */

	struct scsipi_adapter sc_adapter;
	struct scsipi_channel sc_channel;
};

static int  pxdc_match(device_t, cfdata_t, void *);
static void pxdc_attach(device_t, device_t, void *);
static void pxdc_scsi_request(struct scsipi_channel *, scsipi_adapter_req_t,
	void *);
static uint32_t pxdc_datainout_pio(struct pxdc_softc *, struct scsipi_xfer *,
	int, uint32_t);
static void pxdc_status(struct pxdc_softc *, struct scsipi_xfer *);
static void pxdc_done(struct pxdc_softc *, struct scsipi_xfer *);

CFATTACH_DECL_NEW(pxdc, sizeof(struct pxdc_softc),
	pxdc_match, pxdc_attach, NULL, NULL);

int
pxdc_match(device_t parent, cfdata_t cf, void *aux)
{
	struct intio_attach_args *ia = aux;
	bus_space_tag_t iot = ia->ia_bst;
	bus_space_handle_t ioh;

	ia->ia_size = 0x100;

	if (ia->ia_addr != 0xeac500)
		return 0;

	if (badaddr((void *)IIOV(ia->ia_addr)))
		return 0;

	if (bus_space_map(iot, ia->ia_addr, ia->ia_size, 0, &ioh) < 0)
		return 0;

	/* シグネチャしらべる */

	bus_space_unmap(iot, ioh, ia->ia_size);

	return 1;
}

void
pxdc_attach(device_t parent, device_t self, void *aux)
{
	struct pxdc_softc *sc = device_private(self);
	struct intio_attach_args *ia = aux;
	uint32_t data;
	int id;

	sc->sc_dev = self;
	sc->sc_iot = ia->ia_bst;

	intio_map_allocate_region(parent, ia, INTIO_MAP_ALLOCATE);

	if (bus_space_map(sc->sc_iot, ia->ia_addr, ia->ia_size, 0, &sc->sc_ioh)) {
		aprint_normal(": can't map i/o space\n");
		return;
	}

	data = bus_space_read_4(sc->sc_iot, sc->sc_ioh, PXDC_SIG);
	if (data != PXDC_SIGNATURE) {
		aprint_normal(": unknown signature 0x%08x\n", data);
		return;
	}

	aprint_normal(": Pluto-X virtual disk controller\n");

	/* check targets */
	for (id = 0; id < 8; id++) {
		/* Ignore specified ID */
		if ((PXDC_MASK_ID & (1 << id)) != 0) {
			continue;
		}

		data = bus_space_read_1(sc->sc_iot, sc->sc_ioh, PXDC_TGT + id);
		data &= PXDC_TGT_MASK;

		if (data == PXDC_TGT_INIT) {
			sc->sc_id = id;
			continue;
		}
		if (data == PXDC_TGT_HD) {
			/* Obtain the target */
			bus_space_write_1(sc->sc_iot, sc->sc_ioh, PXDC_TGT + id,
			    PXDC_TGT_ENABLE);

			data = bus_space_read_1(sc->sc_iot, sc->sc_ioh,
			    PXDC_TGT + id);
			if ((data & PXDC_TGT_ENABLE) != 0) {
				device_printf(sc->sc_dev,
				    "obtained target (id=%d)\n", id);
			} else {
				device_printf(sc->sc_dev,
				    "can't obtain target (id=%d)\n", id);
			}
		}
	}

	/* Fill in the adapter */
	memset(&sc->sc_adapter, 0, sizeof(sc->sc_adapter));
	memset(&sc->sc_channel, 0, sizeof(sc->sc_channel));
	sc->sc_adapter.adapt_dev = self;
	sc->sc_adapter.adapt_nchannels = 1;
	sc->sc_adapter.adapt_openings = 1;
	sc->sc_adapter.adapt_max_periph = 1;
	sc->sc_adapter.adapt_minphys = minphys;
	sc->sc_adapter.adapt_request = pxdc_scsi_request;

	sc->sc_channel.chan_adapter = &sc->sc_adapter;
	sc->sc_channel.chan_bustype = &scsi_bustype;
	sc->sc_channel.chan_channel = 0;
	sc->sc_channel.chan_flags = SCSIPI_CHAN_NOSETTLE;
	sc->sc_channel.chan_ntargets = 8;
	sc->sc_channel.chan_nluns = 1;
	sc->sc_channel.chan_id = sc->sc_id;

	config_found(self, &sc->sc_channel, scsiprint);
}

static void
pxdc_scsi_request(struct scsipi_channel *chan, scsipi_adapter_req_t req,
	void *arg)
{
	struct pxdc_softc *sc = device_private(chan->chan_adapter->adapt_dev);
	struct scsipi_xfer *xs;
	uint32_t offset;
	uint8_t stat;
	uint8_t phase;

	DPRINTF("%s ", __func__);

	switch (req) {
	case ADAPTER_REQ_RUN_XFER:
		xs = arg;
		xs->error = XS_NOERROR;
		DPRINTF("op=$%x len=%d id=%d\n",
		    (int)xs->cmd->opcode, xs->cmdlen,
		    xs->xs_periph->periph_target);

		/* Reset phase */
		bus_space_write_1(sc->sc_iot, sc->sc_ioh, PXDC_RESET, 0x01);

		/* Select drive */
		bus_space_write_1(sc->sc_iot, sc->sc_ioh, PXDC_DSEL,
		    xs->xs_periph->periph_target);
		stat = bus_space_read_1(sc->sc_iot, sc->sc_ioh, PXDC_STAT);
		if ((stat & PXDC_STAT_MASK) != 0) {
			xs->error = XS_TIMEOUT;	/* どのエラーがいいか */
			pxdc_done(sc, xs);
			return;
		}

		/* TODO: prepare for DMA */

		/* Copy CDB and start command */
		bus_space_write_region_1(sc->sc_iot, sc->sc_ioh,
		    PXDC_CDB, (const uint8_t *)xs->cmd, xs->cmdlen);
		bus_space_write_1(sc->sc_iot, sc->sc_ioh,
		    PXDC_SCMD, xs->cmdlen);

		/* Check error and phase */
		stat = bus_space_read_1(sc->sc_iot, sc->sc_ioh, PXDC_STAT);
		if ((stat & PXDC_STAT_MASK) != 0) {
			xs->error = XS_DRIVER_STUFFUP;
			pxdc_done(sc, xs);
			return;
		}
		phase = stat & PXDC_PHASE_MASK;

		offset = 0;
		while (phase == PXDC_PHASE_DATAIN
		    || phase == PXDC_PHASE_DATAOUT) {
			offset += pxdc_datainout_pio(sc, xs, phase, offset);
			if (xs->error != XS_NOERROR) {
				pxdc_done(sc, xs);
				return;
			}

			/* Check error and phase again */
			stat = bus_space_read_1(sc->sc_iot, sc->sc_ioh,
			    PXDC_STAT);
			if ((stat & PXDC_STAT_MASK) != 0) {
				xs->error = XS_DRIVER_STUFFUP;
				pxdc_done(sc, xs);
				return;
			}
			phase = stat & PXDC_PHASE_MASK;
		}

		if (phase == PXDC_PHASE_STATUS) {
			pxdc_status(sc, xs);
		} else {
			device_printf(sc->sc_dev,
			    "unknown phase 0x%02x\n", phase);
			xs->error = XS_DRIVER_STUFFUP;
		}
		pxdc_done(sc, xs);
		return;

	case ADAPTER_REQ_GROW_RESOURCES:
	case ADAPTER_REQ_SET_XFER_MODE:
		/* XXX Not supported */
		return;
	}
}

/*
 * PIO DataIn/Out.
 * Return the length that is actually accessed in this chunk.
 */
static uint32_t
pxdc_datainout_pio(struct pxdc_softc *sc, struct scsipi_xfer *xs, int phase,
	uint32_t offset)
{
	uint32_t len;

	/* available length in this chunk */
	len = bus_space_read_4(sc->sc_iot, sc->sc_ioh, PXDC_TCNT);
	if (offset + len > xs->datalen) {
		/* Move to the reset phase if buffer is short */
		bus_space_write_1(sc->sc_iot, sc->sc_ioh, PXDC_RESET, 0x01);

		xs->error = XS_RESOURCE_SHORTAGE;
		return 0;
	}

	/* It should be only DATAIN or DATAOUT phase here */
	if (phase == PXDC_PHASE_DATAIN) {
		bus_space_read_multi_1(sc->sc_iot, sc->sc_ioh, PXDC_DATA,
		    xs->data + offset, len);
	} else {
		bus_space_write_multi_1(sc->sc_iot, sc->sc_ioh, PXDC_DATA,
		    xs->data + offset, len);
	}
	xs->resid = xs->datalen - (offset + len);

	return len;
}

void
pxdc_status(struct pxdc_softc *sc, struct scsipi_xfer *xs)
{
	xs->status = bus_space_read_1(sc->sc_iot, sc->sc_ioh, PXDC_SSB);
	DPRINTF("%s 0x%x\n", __func__, xs->status);
}

void
pxdc_done(struct pxdc_softc *sc, struct scsipi_xfer *xs)
{
	DPRINTF("pxdc_done(%d)\n", xs->error);
	/* エラーだったらリセットレジスタに書くこと */
	scsipi_done(xs);
}
