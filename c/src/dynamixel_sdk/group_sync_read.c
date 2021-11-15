/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

#include <stdlib.h>

#if defined(__linux__)
#include "group_sync_read.h"
#elif defined(__APPLE__)
#include "group_sync_read.h"
#elif defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#include "group_sync_read.h"
#endif

typedef struct
{
  uint8_t  id;
  uint8_t  error;  // TODO. not used
  uint8_t  *data;
}DataList;

typedef struct
{
  int         port_num;
  int         protocol_version;

  int         data_list_length;

  uint8_t     last_result;
  uint8_t     is_param_changed;

  uint16_t    start_address;
  uint16_t    data_length;

  DataList   *data_list;
}GroupData;

static GroupData *groupData;
static int g_used_group_num = 0;

static int size(int group_num)
{
  int data_num;
  int real_size = 0;

  for (data_num = 0; data_num < groupData[group_num].data_list_length; data_num++)
  {
    if (groupData[group_num].data_list[data_num].id != NOT_USED_ID)
      real_size++;
  }
  return real_size;
};

static int find(int group_num, int id)
{
  int data_num;

  for (data_num = 0; data_num < groupData[group_num].data_list_length; data_num++)
  {
    if (groupData[group_num].data_list[data_num].id == id)
      break;
  }

  return data_num;
}

int groupSyncRead(int port_num, int protocol_version, uint16_t start_address, uint16_t data_length)
{
  int group_num = 0;

  if (g_used_group_num != 0)
  {
    for (group_num = 0; group_num < g_used_group_num; group_num++)
    {
      if (groupData[group_num].is_param_changed != True
          && groupData[group_num].port_num == port_num
          && groupData[group_num].protocol_version == protocol_version
          && groupData[group_num].start_address == start_address
          && groupData[group_num].data_length == data_length)
        break;
    }
  }

  if (group_num == g_used_group_num)
  {
    g_used_group_num++;
    groupData = (GroupData *)realloc(groupData, g_used_group_num * sizeof(GroupData));
  }

  groupData[group_num].port_num = port_num;
  groupData[group_num].protocol_version = protocol_version;
  groupData[group_num].data_list_length = 0;
  groupData[group_num].last_result = False;
  groupData[group_num].is_param_changed = True;
  groupData[group_num].start_address = start_address;
  groupData[group_num].data_length = data_length;
  groupData[group_num].data_list = 0;

  groupSyncReadClearParam(group_num);

  return group_num;
}

void groupSyncReadMakeParam(int group_num)
{
  int data_num, idx;
  int port_num = groupData[group_num].port_num;

  if (groupData[group_num].protocol_version == 1)
    return;

  if (size(group_num) == 0)
    return;

  packetData[port_num].data_write = (uint8_t*)realloc(packetData[port_num].data_write, size(group_num) * (1) * sizeof(uint8_t)); // ID(1)

  idx = 0;
  for (data_num = 0; data_num < groupData[group_num].data_list_length; data_num++)
  {
    if (groupData[group_num].data_list[data_num].id == NOT_USED_ID)
      continue;

    packetData[port_num].data_write[idx++] = groupData[group_num].data_list[data_num].id;
  }
}

uint8_t groupSyncReadAddParam(int group_num, uint8_t id)
{
  int data_num = 0;

  if (groupData[group_num].protocol_version == 1)
    return False;

  if (id == NOT_USED_ID)
    return False;

  if (groupData[group_num].data_list_length != 0)
    data_num = find(group_num, id);

  if (groupData[group_num].data_list_length == data_num)
  {
    groupData[group_num].data_list_length++;
    groupData[group_num].data_list = (DataList *)realloc(groupData[group_num].data_list, groupData[group_num].data_list_length * sizeof(DataList));

    groupData[group_num].data_list[data_num].id = id;
    groupData[group_num].data_list[data_num].data = (uint8_t *)calloc(groupData[group_num].data_length, sizeof(uint8_t));
  }

  groupData[group_num].is_param_changed = True;
  return True;
}
void groupSyncReadRemoveParam(int group_num, uint8_t id)
{
  int data_num = find(group_num, id);

  if (groupData[group_num].protocol_version == 1)
    return;

  if (groupData[group_num].data_list[data_num].id == NOT_USED_ID)  // NOT exist
    return;

  free(groupData[group_num].data_list[data_num].data);
  groupData[group_num].data_list[data_num].data = 0;

  groupData[group_num].data_list[data_num].id = NOT_USED_ID;

  groupData[group_num].is_param_changed = True;
}
void groupSyncReadClearParam(int group_num)
{
  int data_num = 0;
  int port_num = groupData[group_num].port_num;

  if (groupData[group_num].protocol_version == 1)
    return;

  if (size(group_num) == 0)
    return;

  for (data_num = 0; data_num < groupData[group_num].data_list_length; data_num++)
  {
    free(groupData[group_num].data_list[data_num].data);
    groupData[group_num].data_list[data_num].data = 0;
  }

  free(groupData[group_num].data_list);
  groupData[group_num].data_list = 0;

  free(packetData[port_num].data_write);
  packetData[port_num].data_write = 0;

  groupData[group_num].data_list_length = 0;

  groupData[group_num].is_param_changed = False;
}

void groupSyncReadTxPacket(int group_num)
{
  int port_num = groupData[group_num].port_num;

  if (groupData[group_num].protocol_version == 1)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  if (size(group_num) == 0)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  if (groupData[group_num].is_param_changed == True)
    groupSyncReadMakeParam(group_num);

  syncReadTx(groupData[group_num].port_num,
    groupData[group_num].protocol_version,
    groupData[group_num].start_address,
    groupData[group_num].data_length,
    (size(group_num) * 1));
}

void groupFastSyncReadTxPacket(int group_num)
{
  int port_num = groupData[group_num].port_num;

  if (groupData[group_num].protocol_version == 1)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  if (size(group_num) == 0)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  if (groupData[group_num].is_param_changed == True)
    groupSyncReadMakeParam(group_num);

  fastSyncReadTx(groupData[group_num].port_num,
    groupData[group_num].protocol_version,
    groupData[group_num].start_address,
    groupData[group_num].data_length,
    (size(group_num) * 1));
}

void groupSyncReadRxPacket(int group_num)
{
  int data_num, c;
  int port_num = groupData[group_num].port_num;

  groupData[group_num].last_result = False;

  if (groupData[group_num].protocol_version == 1)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  packetData[groupData[group_num].port_num].communication_result = COMM_RX_FAIL;

  if (size(group_num) == 0)
  {
    packetData[groupData[group_num].port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  for (data_num = 0; data_num < groupData[group_num].data_list_length; data_num++)
  {
    if (groupData[group_num].data_list[data_num].id == NOT_USED_ID)
      continue;

    packetData[port_num].data_read
      = (uint8_t *)realloc(packetData[port_num].data_read, groupData[group_num].data_length * sizeof(uint8_t));

    readRx(groupData[group_num].port_num, groupData[group_num].protocol_version, groupData[group_num].data_length);
    if (packetData[port_num].communication_result != COMM_SUCCESS)
      return;

    for (c = 0; c < groupData[group_num].data_length; c++)
      groupData[group_num].data_list[data_num].data[c] = packetData[port_num].data_read[c];
  }

  if (packetData[port_num].communication_result == COMM_SUCCESS)
    groupData[group_num].last_result = True;
}

void groupFastSyncReadRxPacket(int group_num)
{
  int index = 0;
  int port_num = groupData[group_num].port_num;
  // The size of data in Param + CRC of Fast Sync Read Rx packet
  int data_size = (groupData[group_num].data_length + 4) * groupData[group_num].data_list_length * sizeof(uint8_t) - 1;

  groupData[group_num].last_result = False;

  if (groupData[group_num].protocol_version == 1)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  packetData[groupData[group_num].port_num].communication_result = COMM_RX_FAIL;

  if (size(group_num) == 0)
  {
    packetData[groupData[group_num].port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  // only ONE status packet is received. Parse once and exit.
  packetData[port_num].data_read = (uint8_t *)realloc(packetData[port_num].data_read, data_size);
  // save the stripped packet to data_read buffer (ID + Param + CRC + ERR + ID + Param + CRC + ... + CRC)
  fastSyncReadRx(groupData[group_num].port_num, groupData[group_num].protocol_version, data_size);
  
  if (packetData[port_num].communication_result != COMM_SUCCESS)
  {
    return;
  }

  for (uint8_t index_id = 0; index_id < groupData[group_num].data_list_length; )
  {
    // The first packet that doesn't have ERR. [ID + Param + CRC]
    if (index_id == 0) {
      index = 0;
      if (groupData[group_num].data_list[index_id].id == packetData[port_num].data_read[index])
      {
        index = index + 1;
        for (int loop_for_data = 0; loop_for_data < groupData[group_num].data_length; )
        {
          groupData[group_num].data_list[index_id].data[loop_for_data] = packetData[port_num].data_read[index];
          index = index + 1;
          loop_for_data = loop_for_data + 1;
        }
        // Skip the CRC. CRC check has been done already in the readRX().
        index = index + 2;
      } else {
        // Rx doesn't match with requested ID
        return;
      }
    }
    // Second and after : [ERR + ID + Param + CRC]
    else {
      // Skip the ERR byte
      index = index + 1;
      if (groupData[group_num].data_list[index_id].id == packetData[port_num].data_read[index])
      {
        index = index + 1;
        for (int loop_for_data = 0; loop_for_data < groupData[group_num].data_length; )
        {
          groupData[group_num].data_list[index_id].data[loop_for_data] = packetData[port_num].data_read[index];
          index = index + 1;
          loop_for_data = loop_for_data + 1;
        }
        // Skip the CRC. CRC check has been done already in the readRX().
        index = index + 2;
      } else {
        // Rx doesn't match with requested ID
        return;
      }
    }
    index_id = index_id + 1;
  }

  if (packetData[port_num].communication_result == COMM_SUCCESS)
    groupData[group_num].last_result = True;
}

void groupSyncReadTxRxPacket(int group_num)
{
  int port_num = groupData[group_num].port_num;

  if (groupData[group_num].protocol_version == 1)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  packetData[port_num].communication_result = COMM_TX_FAIL;

  groupSyncReadTxPacket(group_num);
  if (packetData[port_num].communication_result != COMM_SUCCESS)
    return;

  groupSyncReadRxPacket(group_num);
}

void groupFastSyncReadTxRxPacket(int group_num)
{
  int port_num = groupData[group_num].port_num;

  if (groupData[group_num].protocol_version == 1)
  {
    packetData[port_num].communication_result = COMM_NOT_AVAILABLE;
    return;
  }

  packetData[port_num].communication_result = COMM_TX_FAIL;

  groupFastSyncReadTxPacket(group_num);
  if (packetData[port_num].communication_result != COMM_SUCCESS)
    return;

  groupFastSyncReadRxPacket(group_num);
}

uint8_t groupSyncReadIsAvailable(int group_num, uint8_t id, uint16_t address, uint16_t data_length)
{
  int data_num = find(group_num, id);

  if (groupData[group_num].protocol_version == 1 || groupData[group_num].last_result == False || groupData[group_num].data_list[data_num].id == NOT_USED_ID)
    return False;

  if (address < groupData[group_num].start_address || groupData[group_num].start_address + groupData[group_num].data_length - data_length < address) {
    return False;
  }
  return True;
}

uint32_t groupSyncReadGetData(int group_num, uint8_t id, uint16_t address, uint16_t data_length)
{
  int data_num = find(group_num, id);

  if (groupSyncReadIsAvailable(group_num, id, address, data_length) == False)
    return 0;

  switch (data_length)
  {
    case 1:
      return groupData[group_num].data_list[data_num].data[address - groupData[group_num].start_address];

    case 2:
      return DXL_MAKEWORD(
        groupData[group_num].data_list[data_num].data[address - groupData[group_num].start_address],
        groupData[group_num].data_list[data_num].data[address - groupData[group_num].start_address + 1]
      );

    case 4:
      return DXL_MAKEDWORD(
        DXL_MAKEWORD(
          groupData[group_num].data_list[data_num].data[address - groupData[group_num].start_address + 0],
          groupData[group_num].data_list[data_num].data[address - groupData[group_num].start_address + 1]
        ),
        DXL_MAKEWORD(
          groupData[group_num].data_list[data_num].data[address - groupData[group_num].start_address + 2],
          groupData[group_num].data_list[data_num].data[address - groupData[group_num].start_address + 3]
        )
      );

    default:
      return 0;
  }
}
