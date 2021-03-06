// license:GPL-2.0+
// copyright-holders:Couriersud

// ***************************************************************************
//
//    net_lib.cpp
//
// ***************************************************************************

#include "net_lib.h"
#include "netlist/nl_factory.h"
#include "netlist/solver/nld_solver.h"


#define NETLIB_DEVICE_DECL(chip) extern factory::constructor_ptr_t decl_ ## chip
#define LIB_DECL(decl) factory.add( decl () );
#define LIB_ENTRY(nic) { NETLIB_DEVICE_DECL(nic); LIB_DECL(decl_ ## nic) }

namespace netlist
{
namespace devices
{

	void initialize_factory(factory::list_t &factory)
	{
		// The following is from a script which automatically creates
		// the entries.
		// FIXME: the list should be either included or the whole
		// initialize factory code should be created programmatically.
#if (NL_AUTO_DEVICES)
		#include "../generated/lib_entries.hxx"
#else
		LIB_ENTRY(R)
		LIB_ENTRY(POT)
		LIB_ENTRY(POT2)
		LIB_ENTRY(C)
		LIB_ENTRY(L)
		LIB_ENTRY(D)
		LIB_ENTRY(Z)
		LIB_ENTRY(VS)
		LIB_ENTRY(CS)
		LIB_ENTRY(VCVS)
		LIB_ENTRY(VCCS)
		LIB_ENTRY(CCCS)
		LIB_ENTRY(CCVS)
		LIB_ENTRY(LVCCS)
		LIB_ENTRY(opamp)
		LIB_ENTRY(nc_pin)
		LIB_ENTRY(frontier)   // not intended to be used directly
		LIB_ENTRY(function)   // only for macro devices - NO FEEDBACK loops
		LIB_ENTRY(QBJT_EB)
		LIB_ENTRY(QBJT_switch)
		LIB_ENTRY(MOSFET)
		LIB_ENTRY(logic_input_ttl)
		LIB_ENTRY(logic_input)
		LIB_ENTRY(logic_input8)
		LIB_ENTRY(analog_input)
		LIB_ENTRY(log)
		LIB_ENTRY(logD)
		LIB_ENTRY(clock)
		LIB_ENTRY(varclock)
		LIB_ENTRY(extclock)
		LIB_ENTRY(mainclock)
		LIB_ENTRY(gnd)
		LIB_ENTRY(netlistparams)
		LIB_ENTRY(solver)
		LIB_ENTRY(sys_dsw1)
		LIB_ENTRY(sys_dsw2)
		LIB_ENTRY(sys_compd)
		LIB_ENTRY(sys_noise_mt_u)
		LIB_ENTRY(sys_noise_mt_n)
		LIB_ENTRY(switch1)
		LIB_ENTRY(switch2)
		LIB_ENTRY(nicRSFF)
		LIB_ENTRY(nicDelay)
		LIB_ENTRY(2102A)
		LIB_ENTRY(2716)
		LIB_ENTRY(7448)
		LIB_ENTRY(MK28000)
		LIB_ENTRY(7450)
		LIB_ENTRY(7473)
		LIB_ENTRY(7473A)
		LIB_ENTRY(7474)
		LIB_ENTRY(7475_GATE)
		LIB_ENTRY(7477_GATE)
		LIB_ENTRY(7483)
		LIB_ENTRY(7485)
		LIB_ENTRY(7490)
		LIB_ENTRY(7492)
		LIB_ENTRY(7493)
		LIB_ENTRY(7497)
		LIB_ENTRY(74107)
		LIB_ENTRY(74107A)
		LIB_ENTRY(74113)
		LIB_ENTRY(74113A)
		LIB_ENTRY(74121)
		LIB_ENTRY(74123)
		LIB_ENTRY(74125)
		LIB_ENTRY(74126)
		LIB_ENTRY(74153)
		LIB_ENTRY(74161)
		LIB_ENTRY(74161_fixme)
		LIB_ENTRY(74163)
		LIB_ENTRY(74164)
		LIB_ENTRY(74165)
		LIB_ENTRY(74166)
		LIB_ENTRY(74174)
		LIB_ENTRY(74175)
		LIB_ENTRY(74192)
		LIB_ENTRY(74193)
		LIB_ENTRY(74194)
		LIB_ENTRY(74365)
		LIB_ENTRY(74377_GATE)
		LIB_ENTRY(74393)
		//ENTRY(74279,              TTL_74279,              "") // only dip available
		LIB_ENTRY(SN74LS629)
		LIB_ENTRY(82S16)
		LIB_ENTRY(82S115)
		LIB_ENTRY(82S123)
		LIB_ENTRY(82S126)
		LIB_ENTRY(74S287)
		LIB_ENTRY(8277)
		LIB_ENTRY(9310)
		LIB_ENTRY(9314)
		LIB_ENTRY(9316)
		LIB_ENTRY(9321)
		LIB_ENTRY(9322)
		LIB_ENTRY(9334)
		LIB_ENTRY(AM2847)
		LIB_ENTRY(CD4006)
		LIB_ENTRY(CD4013)
		LIB_ENTRY(CD4017)
		LIB_ENTRY(CD4022)
		LIB_ENTRY(CD4020)
		LIB_ENTRY(CD4024)
		LIB_ENTRY(CD4053_GATE)
		LIB_ENTRY(CD4066_GATE)
		LIB_ENTRY(CD4316_GATE)
		LIB_ENTRY(4538)
		LIB_ENTRY(schmitt_trigger)
		LIB_ENTRY(NE555)
		LIB_ENTRY(MC1455P)
		LIB_ENTRY(TMS4800)
		LIB_ENTRY(r2r_dac)
		LIB_ENTRY(tristate)
		LIB_ENTRY(tristate3)
		LIB_ENTRY(9602)
		LIB_ENTRY(MM5837)
#endif
	}

} //namespace devices
} // namespace netlist

