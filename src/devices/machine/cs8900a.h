// license:GPL-2.0+
// copyright-holders:Rhett Aultman
/*************************************************************************

    CS8900A ethernet controller implementation

    by Rhett Aultman <roadriverrail@gmail.com>
    ported to MAME from VICE Project (https://sourceforge.net/p/vice-emu/)
    VICE CS8900 code by Spiro Trikaliotis <Spiro.Trikaliotis@gmx.de>

**************************************************************************/

#ifndef MAME_MACHINE_CS8900A_H
#define MAME_MACHINE_CS8900A_H

#pragma once

#include <queue>

#define TFE_COUNT_IO_REGISTER 0x10 /* we have 16 I/O register */
#define MAX_PACKETPAGE_ARRAY 0x1000 /* 4 KB */

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
	u8 tfe_ia_mac[6];

	u32 tfe_hash_mask[2];

	// CS8900A IO Registers
	u8 tfe[TFE_COUNT_IO_REGISTER];

	// CS8900A PacketPage
	u8 tfe_packetpage[MAX_PACKETPAGE_ARRAY];
	u16 tfe_packetpage_ptr;

	/* reveiver setup */
	u16 tfe_recv_control;	 /* copy of CC_RXCTL (contains all bits below) */
	int tfe_recv_broadcast;	 /* broadcast */
	int tfe_recv_mac;		 /* individual address (IA) */
	int tfe_recv_multicast;	 /* multicast if address passes the hash filter */
	int tfe_recv_correct;	 /* accept correct frames */
	int tfe_recv_promiscuous; /* promiscuous mode */
	int tfe_recv_hashfilter;	 /* accept if IA passes the hash filter */



	u16 tx_buffer;
	u16 rx_buffer;

	u16 tx_count;
	u16 rx_count;
	u16 tx_length;
	u16 rx_length;

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
