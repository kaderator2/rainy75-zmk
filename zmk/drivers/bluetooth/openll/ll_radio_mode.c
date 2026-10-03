/*
 * Copyright (c) 2026 scholzri
 * SPDX-License-Identifier: Apache-2.0
 *
 * Register-mode bookkeeping of ll_radio (rules in ll_radio_mode.h).
 */
#include <string.h>

#include "ll_radio_mode.h"

void ll_radio_mode_reset(struct ll_radio_mode *m)
{
	memset(m, 0, sizeof(*m));
}

bool ll_radio_mode_conn_init(struct ll_radio_mode *m, uint8_t cur_ctrl1, uint8_t cur_rxtcrc)
{
	if (m->conn) {
		return false;
	}
	if (!m->snap) {
		/* only ever from advertising registers: the first setup */
		m->adv_ctrl1 = cur_ctrl1;
		m->adv_rxtcrc = cur_rxtcrc;
		m->snap = true;
	}
	m->conn = true;
	return true;
}

bool ll_radio_mode_conn_ready(const struct ll_radio_mode *m)
{
	return m->conn;
}

bool ll_radio_mode_restore(struct ll_radio_mode *m, bool keep_conn, uint8_t *ctrl1,
			   uint8_t *rxtcrc)
{
	if (!keep_conn) {
		m->conn = false;
	}
	*ctrl1 = m->adv_ctrl1;
	*rxtcrc = m->adv_rxtcrc;
	return m->snap;
}

bool ll_radio_mode_adv_regs(const struct ll_radio_mode *m, uint8_t *ctrl1, uint8_t *rxtcrc)
{
	*ctrl1 = m->adv_ctrl1;
	*rxtcrc = m->adv_rxtcrc;
	return m->snap;
}
