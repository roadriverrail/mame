// license:GPL-2.0+
// copyright-holders:Rhett Aultman
/*************************************************************************

    CS8900A ethernet controller implementation

    by Rhett Aultman
    modified from The Final Ethernet (TFE)
    by Spiro Trikaliotis, Christian Vogelgsang

**************************************************************************/

#ifndef MAME_MACHINE_CS8900A_H
#define MAME_MACHINE_CS8900A_H

#pragma once

#include <queue>

/***************************************************************************
    TYPE DEFINITIONS
***************************************************************************/

class cs8900a_device : public device_t, public device_network_interface
{
public:
	cs8900a_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock);
	u8 read(u16 address);
	void write(u16 address, u8 data);

protected:
	cs8900a_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;
	virtual void device_reset() override;

	// device_network_interface overrides
	virtual int recv_start_cb(u8 *buf, int length) override;

private:
	int should_activate;
	int init_tfe_flag;
	u8 tfe_ia_mac[6];

	u32 tfe_hash_mask[2];

	/* reveiver setup */
	u16 tfe_recv_control;	 /* copy of CC_RXCTL (contains all bits below) */
	int tfe_recv_broadcast;	 /* broadcast */
	int tfe_recv_mac;		 /* individual address (IA) */
	int tfe_recv_multicast;	 /* multicast if address passes the hash filter */
	int tfe_recv_correct;	 /* accept correct frames */
	int tfe_recv_promiscuous; /* promiscuous mode */
	int tfe_recv_hashfilter;	 /* accept if IA passes the hash filter */

	/* Flag: Can we even use TFE, or is the hardware not available? */
	int tfe_cannot_use;

	/* Flag: Do we have the TFE enabled?  */
	int tfe_enabled;

	/* Flag: Do we use the "original" memory map or the memory map of the RR-Net? */
	int tfe_as_rr_net;

	char *tfe_interface;

	u8 *tfe;

	u16 tx_buffer;
	u16 rx_buffer;

	u16 tx_count;
	u16 rx_count;
	u16 tx_length;
	u16 rx_length;

#define TFE_TX_IDLE 0
#define TFE_TX_GOT_CMD 1
#define TFE_TX_GOT_LEN 2
#define TFE_TX_READ_BUSST 3

#define TFE_RX_IDLE 0
#define TFE_RX_GOT_FRAME 1

	/* tranceiver state */
	int tx_state;
	int rx_state;
	int tx_enabled;
	int rx_enabled;

	int rxevent_read_mask; /* set if L and/or H u8 was read in RXEVENT? */

	//CRC support stuff
	unsigned long crc32_table[256];
	int crc32_is_initialized;

	//The frame queue
	std::queue<std::vector<u8>> m_frame_queue;

	void tfe_set_tx_status(int ready, int error);
	void tfe_set_receiver(int enabled);
	void tfe_set_transmitter(int enabled);
	int tfe_deactivate(void);
	int tfe_deactivate_i(void);
	int tfe_activate(void);
	void tfe_shutdown(void);
	int tfe_should_accept(unsigned char *buffer, int length, int *phashed, int *phash_index, 
                      int *pcorrect_mac, int *pbroadcast, int *pmulticast);
	u16 tfe_receive(void);
	void tfe_write_tx_buffer(u8 value,int odd_address);
	u8 tfe_read_rx_buffer(int odd_address);
	void tfe_sideeffects_write_pp(u16 ppaddress, int odd_address);
	void tfe_sideeffects_read_pp(u16 ppaddress,int odd_address);
	u16 tfe_read_register(u16 ppaddress);
	void tfe_write_register(u16 ppaddress,u16 value);
	void tfe_auto_incr_pp_ptr(void);
	u8 tfe_read(u16 io_address);
	void tfe_store(u16 io_address, u8 var);
	unsigned long crc32_buf(const char *buffer, unsigned int len);
};

DECLARE_DEVICE_TYPE(CS8900A, cs8900a_device)

	/***************************************************************************
    DEVICE CONFIGURATION MACROS
***************************************************************************/

#endif // MAME_MACHINE_GM8900A_H
