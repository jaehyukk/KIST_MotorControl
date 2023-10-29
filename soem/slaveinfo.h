/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : slaveinfo [ifname] [-sdo] [-map]
 * Ifname is NIC interface, f.e. eth0.
 * Optional -sdo to display CoE object dictionary.
 * Optional -map to display slave PDO mapping
 *
 * This shows the configured slave data.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#ifndef _slaveinfo_
#define _slaveinfo_

#ifdef __cplusplus
extern "C"
{
#endif

char* dtype2string(uint16 dtype, uint16 bitlen);
char* otype2string(uint16 otype);
char* access2string(uint16 access);
char* SDO2string(uint16 slave, uint16 index, uint8 subidx, uint16 dtype);
int si_PDOassign(uint16 slave, uint16 PDOassign, int mapoffset, int bitoffset);
int si_map_sdo(int slave);
int si_siiPDO(uint16 slave, uint8 t, int mapoffset, int bitoffset);
int si_map_sii(int slave);
void si_sdo(int cnt);
void slaveinfo(char *ifname);


#ifdef __cplusplus
}
#endif

#endif
