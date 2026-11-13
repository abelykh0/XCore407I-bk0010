// SPDX-License-Identifier: MIT
// XCore407I read-only MTP view of the LittleFS root directory.

#include "tusb.h"
#include "class/mtp/mtp_device.h"
#include <string.h>

#define MTP_STORAGE_ID 0x00010001u
#define MTP_MAX_OBJECTS 16u
#define MTP_READ_LIMIT 4084u
#define MTP_WORKSPACE_READ_OFFSET 1024u

extern bool xcore_mtp_refresh(void);
extern uint32_t xcore_mtp_object_count(void);
extern void xcore_mtp_storage_stats(uint32_t *total, uint32_t *free);
extern bool xcore_mtp_object_info(uint32_t handle, uint16_t *name,
                                  uint32_t max_name_units, uint32_t *size);
extern int32_t xcore_mtp_load_object(uint32_t handle, uint8_t *destination,
                                     uint32_t capacity);
extern uint8_t *xcore_mtp_workspace(void);

typedef MTP_STORAGE_INFO_STRUCT(6, 5) xcore_mtp_storage_info_t;

static xcore_mtp_storage_info_t storage_info = {
  .storage_type = MTP_STORAGE_TYPE_FIXED_ROM,
  .filesystem_type = MTP_FILESYSTEM_TYPE_GENERIC_HIERARCHICAL,
  .access_capability = MTP_ACCESS_CAPABILITY_READ_ONLY_WITHOUT_OBJECT_DELETION,
  .max_capacity_in_bytes = 0,
  .free_space_in_bytes = 0,
  .free_space_in_objects = 0,
  .storage_description = {
    .count = 6,
    .utf16 = { 'X', 'C', 'o', 'r', 'e', 0 }
  },
  .volume_identifier = {
    .count = 5,
    .utf16 = { 'N', 'A', 'N', 'D', 0 }
  }
};

static bool session_open;
static uint32_t active_object_handle;
static uint32_t active_object_offset;
static uint32_t active_object_length;
static bool active_partial_object;

static uint8_t *mtp_read_buffer(void) {
  return xcore_mtp_workspace() + MTP_WORKSPACE_READ_OFFSET;
}

static int32_t send_data(tud_mtp_cb_data_t *cb_data) {
  return tud_mtp_data_send(&cb_data->io_container) ? 0 : MTP_RESP_DEVICE_BUSY;
}

static int32_t send_simple_response(tud_mtp_cb_data_t *cb_data, uint16_t code) {
  mtp_container_info_t *io = &cb_data->io_container;
  io->header->len = sizeof(mtp_container_header_t);
  io->header->code = code;
  return tud_mtp_response_send(io) ? 0 : -1;
}

static int32_t get_device_info(tud_mtp_cb_data_t *cb_data) {
  mtp_container_info_t *io = &cb_data->io_container;
  (void) mtp_container_add_cstring(io, "XCore407I");
  (void) mtp_container_add_cstring(io, "XCore407I BK-0010");
  (void) mtp_container_add_cstring(io, "1.0");
  (void) mtp_container_add_cstring(io, "XCORE407I");
  return send_data(cb_data);
}

static int32_t open_close_session(tud_mtp_cb_data_t *cb_data) {
  const uint16_t operation = cb_data->command_container->header.code;
  if (operation == MTP_OP_OPEN_SESSION) {
    if (session_open) {
      return MTP_RESP_SESSION_ALREADY_OPEN;
    }
    session_open = true;
    return MTP_RESP_OK;
  }

  if (!session_open) {
    return MTP_RESP_SESSION_NOT_OPEN;
  }
  session_open = false;
  return MTP_RESP_OK;
}

static int32_t get_storage_ids(tud_mtp_cb_data_t *cb_data) {
  const uint32_t ids[1] = { MTP_STORAGE_ID };
  (void) mtp_container_add_auint32(&cb_data->io_container, 1, ids);
  return send_data(cb_data);
}

static int32_t get_storage_info(tud_mtp_cb_data_t *cb_data) {
  const uint32_t storage_id = cb_data->command_container->params[0];
  if (storage_id != MTP_STORAGE_ID) {
    return MTP_RESP_INVALID_STORAGE_ID;
  }

  uint32_t total = 0;
  uint32_t free = 0;
  xcore_mtp_storage_stats(&total, &free);
  storage_info.max_capacity_in_bytes = total;
  storage_info.free_space_in_bytes = free;
  storage_info.free_space_in_objects = MTP_MAX_OBJECTS - xcore_mtp_object_count();
  mtp_container_add_raw(&cb_data->io_container, &storage_info, sizeof(storage_info));
  return send_data(cb_data);
}

static int32_t get_num_objects(tud_mtp_cb_data_t *cb_data) {
  const uint32_t storage_id = cb_data->command_container->params[0];
  if (storage_id != 0xFFFFFFFFu && storage_id != MTP_STORAGE_ID) {
    return MTP_RESP_INVALID_STORAGE_ID;
  }
  const uint32_t count = xcore_mtp_object_count();
  (void) mtp_container_add_uint32(&cb_data->io_container, count);
  return send_data(cb_data);
}

static int32_t get_object_handles(tud_mtp_cb_data_t *cb_data) {
  const mtp_container_command_t *command = cb_data->command_container;
  const uint32_t storage_id = command->params[0];
  const uint32_t parent = command->params[2];
  if (storage_id != 0xFFFFFFFFu && storage_id != MTP_STORAGE_ID) {
    return MTP_RESP_INVALID_STORAGE_ID;
  }
  if (parent != 0 && parent != 0xFFFFFFFFu) {
    return MTP_RESP_INVALID_PARENT_OBJECT;
  }

  uint32_t handles[MTP_MAX_OBJECTS];
  const uint32_t count = xcore_mtp_object_count();
  for (uint32_t i = 0; i < count; i++) {
    handles[i] = i + 1;
  }
  (void) mtp_container_add_auint32(&cb_data->io_container, count, handles);
  return send_data(cb_data);
}

static int32_t get_object_info(tud_mtp_cb_data_t *cb_data) {
  const uint32_t handle = cb_data->command_container->params[0];
  uint16_t name[65] = { 0 };
  uint32_t size = 0;
  if (!xcore_mtp_object_info(handle, name, TU_ARRAY_SIZE(name), &size)) {
    return MTP_RESP_INVALID_OBJECT_HANDLE;
  }

  mtp_object_info_header_t info = {
    .storage_id = MTP_STORAGE_ID,
    .object_format = MTP_OBJ_FORMAT_UNDEFINED,
    .protection_status = MTP_PROTECTION_STATUS_READ_ONLY,
    .object_compressed_size = size,
    .thumb_format = MTP_OBJ_FORMAT_UNDEFINED,
    .thumb_compressed_size = 0,
    .thumb_pix_width = 0,
    .thumb_pix_height = 0,
    .image_pix_width = 0,
    .image_pix_height = 0,
    .image_bit_depth = 0,
    .parent_object = 0,
    .association_type = MTP_ASSOCIATION_UNDEFINED,
    .association_desc = 0,
    .sequence_number = 0
  };

  mtp_container_info_t *io = &cb_data->io_container;
  (void) mtp_container_add_raw(io, &info, sizeof(info));
  (void) mtp_container_add_string(io, name);
  (void) mtp_container_add_cstring(io, "");
  (void) mtp_container_add_cstring(io, "");
  (void) mtp_container_add_cstring(io, "");
  return send_data(cb_data);
}

static int32_t start_object_data(tud_mtp_cb_data_t *cb_data, bool partial) {
  if (!session_open) {
    return MTP_RESP_SESSION_NOT_OPEN;
  }

  const mtp_container_command_t *command = cb_data->command_container;
  const uint32_t handle = command->params[0];
  uint32_t size = 0;
  if (!xcore_mtp_object_info(handle, NULL, 0, &size)) {
    return MTP_RESP_INVALID_OBJECT_HANDLE;
  }
  if (size > MTP_READ_LIMIT) {
    return MTP_RESP_OBJECT_TOO_LARGE;
  }

  const int32_t loaded = xcore_mtp_load_object(handle, mtp_read_buffer(), MTP_READ_LIMIT);
  if (loaded < 0 || (uint32_t) loaded != size) {
    return MTP_RESP_GENERAL_ERROR;
  }

  uint32_t offset = 0;
  uint32_t length = size;
  if (partial) {
    offset = command->params[1];
    const uint32_t requested = command->params[2];
    if (offset > size) {
      return MTP_RESP_INVALID_PARAMETER;
    }
    length = tu_min32(size - offset, requested);
  }

  active_object_handle = handle;
  active_object_offset = offset;
  active_object_length = length;
  active_partial_object = partial;

  mtp_container_info_t *io = &cb_data->io_container;
  io->header->len = sizeof(mtp_container_header_t) + length;
  const uint32_t first = tu_min32(length, io->payload_bytes);
  if (first != 0) {
    memcpy(io->payload, mtp_read_buffer() + offset, first);
  }
  return tud_mtp_data_send(io) ? 0 : MTP_RESP_DEVICE_BUSY;
}

static int32_t dispatch_operation(tud_mtp_cb_data_t *cb_data) {
  const uint16_t operation = cb_data->command_container->header.code;
  int32_t response;

  switch (operation) {
    case MTP_OP_GET_DEVICE_INFO:
      response = get_device_info(cb_data);
      break;
    case MTP_OP_OPEN_SESSION:
    case MTP_OP_CLOSE_SESSION:
      response = open_close_session(cb_data);
      break;
    case MTP_OP_GET_STORAGE_IDS:
      response = get_storage_ids(cb_data);
      break;
    case MTP_OP_GET_STORAGE_INFO:
      response = get_storage_info(cb_data);
      break;
    case MTP_OP_GET_NUM_OBJECTS:
      response = get_num_objects(cb_data);
      break;
    case MTP_OP_GET_OBJECT_HANDLES:
      response = session_open ? get_object_handles(cb_data) : MTP_RESP_SESSION_NOT_OPEN;
      break;
    case MTP_OP_GET_OBJECT_INFO:
      response = session_open ? get_object_info(cb_data) : MTP_RESP_SESSION_NOT_OPEN;
      break;
    case MTP_OP_GET_OBJECT:
      if (cb_data->phase == MTP_PHASE_COMMAND) {
        response = start_object_data(cb_data, false);
      } else {
        const uint32_t sent = cb_data->total_xferred_bytes - sizeof(mtp_container_header_t);
        if (sent >= active_object_length) {
          response = 0;
        } else {
          const uint32_t count = tu_min32(
              active_object_length - sent, cb_data->io_container.payload_bytes);
          memcpy(cb_data->io_container.payload,
                 mtp_read_buffer() + active_object_offset + sent, count);
          response = tud_mtp_data_send(&cb_data->io_container) ? 0 : -1;
        }
      }
      break;
    case MTP_OP_GET_PARTIAL_OBJECT:
      if (cb_data->phase == MTP_PHASE_COMMAND) {
        response = start_object_data(cb_data, true);
      } else {
        const uint32_t sent = cb_data->total_xferred_bytes - sizeof(mtp_container_header_t);
        if (sent >= active_object_length) {
          response = 0;
        } else {
          const uint32_t count = tu_min32(
              active_object_length - sent, cb_data->io_container.payload_bytes);
          memcpy(cb_data->io_container.payload,
                 mtp_read_buffer() + active_object_offset + sent, count);
          response = tud_mtp_data_send(&cb_data->io_container) ? 0 : -1;
        }
      }
      break;
    case MTP_OP_GET_DEVICE_PROP_DESC: {
      const uint16_t property = (uint16_t) cb_data->command_container->params[0];
      if (property != MTP_DEV_PROP_DEVICE_FRIENDLY_NAME) {
        response = MTP_RESP_PARAMETER_NOT_SUPPORTED;
        break;
      }
      mtp_device_prop_desc_header_t desc = {
        .device_property_code = MTP_DEV_PROP_DEVICE_FRIENDLY_NAME,
        .datatype = MTP_DATA_TYPE_STR,
        .get_set = MTP_MODE_GET
      };
      mtp_container_info_t *io = &cb_data->io_container;
      (void) mtp_container_add_raw(io, &desc, sizeof(desc));
      (void) mtp_container_add_cstring(io, "XCore407I BK-0010");
      (void) mtp_container_add_cstring(io, "XCore407I BK-0010");
      (void) mtp_container_add_uint8(io, 0);
      response = send_data(cb_data);
      break;
    }
    case MTP_OP_GET_DEVICE_PROP_VALUE:
      if ((uint16_t) cb_data->command_container->params[0] !=
          MTP_DEV_PROP_DEVICE_FRIENDLY_NAME) {
        response = MTP_RESP_PARAMETER_NOT_SUPPORTED;
      } else {
        (void) mtp_container_add_cstring(&cb_data->io_container, "XCore407I BK-0010");
        response = send_data(cb_data);
      }
      break;
    case MTP_OP_DELETE_OBJECT:
    case MTP_OP_SEND_OBJECT_INFO:
    case MTP_OP_SEND_OBJECT:
      response = MTP_RESP_ACCESS_DENIED;
      break;
    default:
      response = MTP_RESP_OPERATION_NOT_SUPPORTED;
      break;
  }

  if (response > MTP_RESP_UNDEFINED) {
    return send_simple_response(cb_data, (uint16_t) response);
  }
  return response;
}

void xcore_tud_mtp_reset_session(void) {
  session_open = false;
  active_object_handle = 0;
  active_object_offset = 0;
  active_object_length = 0;
  active_partial_object = false;
}

bool tud_mtp_request_cancel_cb(tud_mtp_request_cb_data_t *cb_data) {
  (void) cb_data;
  active_object_handle = 0;
  active_object_length = 0;
  return true;
}

bool tud_mtp_request_device_reset_cb(tud_mtp_request_cb_data_t *cb_data) {
  (void) cb_data;
  session_open = false;
  active_object_handle = 0;
  active_object_length = 0;
  return true;
}

int32_t tud_mtp_request_get_device_status_cb(tud_mtp_request_cb_data_t *cb_data) {
  if (cb_data->bufsize < 4) {
    return -1;
  }
  uint16_t *status = (uint16_t *) cb_data->buf;
  status[0] = 4;
  status[1] = MTP_RESP_OK;
  return 4;
}

int32_t tud_mtp_command_received_cb(tud_mtp_cb_data_t *cb_data) {
  return dispatch_operation(cb_data);
}

int32_t tud_mtp_data_xfer_cb(tud_mtp_cb_data_t *cb_data) {
  const int32_t result = dispatch_operation(cb_data);
  return result < 0 ? result : 0;
}

int32_t tud_mtp_data_complete_cb(tud_mtp_cb_data_t *cb_data) {
  mtp_container_info_t *response = &cb_data->io_container;
  response->header->len = sizeof(mtp_container_header_t);
  if (active_partial_object) {
    (void) mtp_container_add_uint32(response, active_object_length);
  }
  response->header->code = (cb_data->xfer_result == XFER_RESULT_SUCCESS)
      ? MTP_RESP_OK : MTP_RESP_GENERAL_ERROR;
  return tud_mtp_response_send(response) ? 0 : -1;
}

int32_t tud_mtp_response_complete_cb(tud_mtp_cb_data_t *cb_data) {
  (void) cb_data;
  active_object_handle = 0;
  active_object_offset = 0;
  active_object_length = 0;
  active_partial_object = false;
  return 0;
}
