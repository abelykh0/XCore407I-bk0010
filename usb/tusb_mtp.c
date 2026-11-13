// SPDX-License-Identifier: MIT
// XCore407I MTP access to the LittleFS root directory.

#include "tusb.h"
#include "class/mtp/mtp_device.h"
#include <string.h>

#define MTP_STORAGE_ID 0x00010001u
#define MTP_UPLOAD_NAME_MAX 220u
#define MTP_OBJECT_INFO_FIXED_SIZE 52u
#define MTP_UPLOAD_INFO_MAX 600u

extern bool xcore_mtp_refresh(void);
extern uint32_t xcore_mtp_object_count(void);
extern void xcore_mtp_storage_stats(uint32_t *total, uint32_t *free);
extern bool xcore_mtp_object_info(uint32_t handle, uint16_t *name,
                                  uint32_t max_name_units, uint32_t *size);
extern int32_t xcore_mtp_read_object_chunk(uint32_t handle, uint32_t offset,
                                            uint8_t *destination, uint32_t capacity);
extern uint32_t xcore_mtp_find_object(const uint8_t *name, uint32_t name_len);
extern bool xcore_mtp_begin_upload(const uint8_t *name, uint32_t name_len, uint32_t expected_size);
extern bool xcore_mtp_write_upload_chunk(uint32_t offset, const uint8_t *data, uint32_t len);
extern bool xcore_mtp_finish_upload(void);
extern void xcore_mtp_abort_upload(void);

typedef MTP_STORAGE_INFO_STRUCT(6, 5) xcore_mtp_storage_info_t;

static xcore_mtp_storage_info_t storage_info = {
  .storage_type = MTP_STORAGE_TYPE_FIXED_RAM,
  .filesystem_type = MTP_FILESYSTEM_TYPE_GENERIC_HIERARCHICAL,
  .access_capability = MTP_ACCESS_CAPABILITY_READ_WRITE,
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
static uint8_t upload_info[MTP_UPLOAD_INFO_MAX];
static uint32_t upload_info_len;
static uint32_t upload_expected_size;
static bool upload_info_valid;
static bool upload_data_active;
static uint32_t active_handles_count;
static uint32_t send_object_info_params[3];

// GET_OBJECT reads are requested by the USB callback but executed by the main
// loop. The saved container points into TinyUSB's persistent MTP endpoint buffer.
static volatile bool object_read_pending;
static volatile bool object_read_failed;
static volatile bool object_read_ready;
static mtp_container_info_t object_read_io;
static uint32_t object_read_handle;
static uint32_t object_read_offset;
static uint32_t object_read_count;

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
  (void) mtp_container_add_cstring(io, "XCore407I BK-0010 MTP");
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
    // Initialize the existing LittleFS volume only after the host has
    // discovered the USB device and explicitly opened an MTP session.
    // This read-only refresh also caches the first root file for later reads.
    if (!xcore_mtp_refresh()) {
      return MTP_RESP_GENERAL_ERROR;
    }
    session_open = true;
    return MTP_RESP_OK;
  }

  if (!session_open) {
    return MTP_RESP_SESSION_NOT_OPEN;
  }
  session_open = false;
  xcore_mtp_close_file();
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
  storage_info.free_space_in_objects = 0xFFFFFFFFu;
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

  const uint32_t count = xcore_mtp_object_count();
  active_handles_count = count;
  mtp_container_info_t *io = &cb_data->io_container;
  io->header->len = sizeof(mtp_container_header_t);
  (void) mtp_container_add_uint32(io, count);
  const uint32_t first_count = tu_min32(count, (io->payload_bytes - 4u) / 4u);
  for (uint32_t i = 0; i < first_count; i++) {
    const uint32_t handle = i + 1u;
    memcpy(io->payload + 4u + i * 4u, &handle, sizeof(handle));
  }
  io->header->len = sizeof(mtp_container_header_t) + 4u + count * 4u;
  return tud_mtp_data_send(io) ? 0 : MTP_RESP_DEVICE_BUSY;
}

static int32_t get_object_info(tud_mtp_cb_data_t *cb_data) {
  const uint32_t handle = cb_data->command_container->params[0];
  uint16_t name[MTP_UPLOAD_NAME_MAX + 1u] = { 0 };
  uint32_t size = 0;
  if (!xcore_mtp_object_info(handle, name, TU_ARRAY_SIZE(name), &size)) {
    return MTP_RESP_INVALID_OBJECT_HANDLE;
  }

  mtp_object_info_header_t info = {
    .storage_id = MTP_STORAGE_ID,
    .object_format = MTP_OBJ_FORMAT_UNDEFINED,
    .protection_status = MTP_PROTECTION_STATUS_NO_PROTECTION,
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
  if (first == 0) {
    return tud_mtp_data_send(io) ? 0 : MTP_RESP_DEVICE_BUSY;
  }

  object_read_io = *io;
  object_read_handle = handle;
  object_read_offset = offset;
  object_read_count = first;
  object_read_failed = false;
  object_read_ready = false;
  __asm volatile ("" ::: "memory");
  object_read_pending = true;
  return 0;
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
      if (cb_data->phase == MTP_PHASE_COMMAND) {
        response = session_open ? get_object_handles(cb_data) : MTP_RESP_SESSION_NOT_OPEN;
      } else {
        const uint32_t data_offset = cb_data->total_xferred_bytes - sizeof(mtp_container_header_t);
        if (data_offset >= 4u + active_handles_count * 4u) {
          response = 0;
        } else {
          const uint32_t first_handle_offset = data_offset <= 4u ? 0u : (data_offset - 4u) / 4u;
          const uint32_t bytes_already = data_offset <= 4u ? 4u - data_offset : 0u;
          uint8_t *out = cb_data->io_container.payload;
          uint32_t capacity = cb_data->io_container.payload_bytes;
          if (bytes_already != 0u) {
            memset(out, 0, bytes_already);
            out += bytes_already;
            capacity -= bytes_already;
          }
          const uint32_t count = tu_min32(
              active_handles_count - first_handle_offset, capacity / 4u);
          for (uint32_t i = 0; i < count; i++) {
            const uint32_t handle = first_handle_offset + i + 1u;
            memcpy(out + i * 4u, &handle, sizeof(handle));
          }
          response = tud_mtp_data_send(&cb_data->io_container) ? 0 : -1;
        }
      }
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
          object_read_io = cb_data->io_container;
          object_read_handle = active_object_handle;
          object_read_offset = active_object_offset + sent;
          object_read_count = count;
          object_read_failed = false;
          object_read_ready = false;
          __asm volatile ("" ::: "memory");
          object_read_pending = true;
          response = 0;
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
          object_read_io = cb_data->io_container;
          object_read_handle = active_object_handle;
          object_read_offset = active_object_offset + sent;
          object_read_count = count;
          object_read_failed = false;
          object_read_ready = false;
          __asm volatile ("" ::: "memory");
          object_read_pending = true;
          response = 0;
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
      (void) mtp_container_add_cstring(io, "XCore407I BK-0010 MTP");
      (void) mtp_container_add_cstring(io, "XCore407I BK-0010 MTP");
      (void) mtp_container_add_uint8(io, 0);
      response = send_data(cb_data);
      break;
    }
    case MTP_OP_GET_DEVICE_PROP_VALUE:
      if ((uint16_t) cb_data->command_container->params[0] !=
          MTP_DEV_PROP_DEVICE_FRIENDLY_NAME) {
        response = MTP_RESP_PARAMETER_NOT_SUPPORTED;
      } else {
        (void) mtp_container_add_cstring(&cb_data->io_container, "XCore407I BK-0010 MTP");
        response = send_data(cb_data);
      }
      break;
    case MTP_OP_SEND_OBJECT_INFO:
      if (!session_open) {
        response = MTP_RESP_SESSION_NOT_OPEN;
      } else if (cb_data->command_container->params[0] != MTP_STORAGE_ID) {
        response = MTP_RESP_INVALID_STORAGE_ID;
      } else if (cb_data->command_container->params[1] != 0 &&
                 cb_data->command_container->params[1] != 0xFFFFFFFFu) {
        response = MTP_RESP_INVALID_PARENT_OBJECT;
      } else {
        upload_info_len = 0;
        upload_info_valid = false;
        response = tud_mtp_data_receive(&cb_data->io_container) ? 0 : MTP_RESP_DEVICE_BUSY;
      }
      break;
    case MTP_OP_SEND_OBJECT:
      if (!session_open) {
        response = MTP_RESP_SESSION_NOT_OPEN;
      } else if (!upload_info_valid) {
        response = MTP_RESP_NO_VALID_OBJECTINFO;
      } else {
        upload_data_active = true;
        response = tud_mtp_data_receive(&cb_data->io_container) ? 0 : MTP_RESP_DEVICE_BUSY;
      }
      break;
    case MTP_OP_DELETE_OBJECT:
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

// Called from the MTP-mode main loop. NAND/LittleFS access therefore never
// runs in the TinyUSB endpoint callback or OTG_HS interrupt.
bool xcore_mtp_service_read_prepare(void) {
  if (!object_read_pending) {
    return false;
  }
  if (object_read_ready) {
    return true;
  }
  __asm volatile ("" ::: "memory");

  const int32_t read = xcore_mtp_read_object_chunk(
      object_read_handle, object_read_offset,
      object_read_io.payload, object_read_count);

  // Cancel or Device Reset can arrive while LittleFS is being read.
  __asm volatile ("" ::: "memory");
  if (!object_read_pending) {
    return false;
  }

  object_read_failed = read < 0 || (uint32_t)read != object_read_count;
  object_read_ready = true;
  return true;
}

void xcore_mtp_service_read_submit(void) {
  if (!object_read_pending || !object_read_ready) {
    return;
  }
  __asm volatile ("" ::: "memory");

  bool submitted;
  if (object_read_failed) {
    object_read_io.header->len = sizeof(mtp_container_header_t);
    object_read_io.header->code = MTP_RESP_GENERAL_ERROR;
    submitted = tud_mtp_response_send(&object_read_io);
  } else {
    submitted = tud_mtp_data_send(&object_read_io);
  }

  // A refused submission is retried next loop. Keep the already-read chunk.
  if (submitted) {
    object_read_pending = false;
    object_read_failed = false;
    object_read_ready = false;
  }
}

void xcore_tud_mtp_reset_session(void) {
  xcore_mtp_abort_upload();
  upload_info_len = 0;
  upload_expected_size = 0;
  upload_info_valid = false;
  upload_data_active = false;
  session_open = false;
  active_object_handle = 0;
  active_object_offset = 0;
  active_object_length = 0;
  active_partial_object = false;
  object_read_pending = false;
  object_read_failed = false;
  object_read_ready = false;
  xcore_mtp_close_file();
}

bool tud_mtp_request_cancel_cb(tud_mtp_request_cb_data_t *cb_data) {
  (void) cb_data;
  xcore_mtp_abort_upload();
  upload_info_valid = false;
  upload_data_active = false;
  active_object_handle = 0;
  active_object_length = 0;
  object_read_pending = false;
  object_read_failed = false;
  object_read_ready = false;
  xcore_mtp_close_file();
  return true;
}

bool tud_mtp_request_device_reset_cb(tud_mtp_request_cb_data_t *cb_data) {
  (void) cb_data;
  xcore_mtp_abort_upload();
  upload_info_valid = false;
  upload_data_active = false;
  session_open = false;
  active_object_handle = 0;
  active_object_length = 0;
  object_read_pending = false;
  object_read_failed = false;
  object_read_ready = false;
  xcore_mtp_close_file();
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


static bool parse_upload_object_info(void) {
  if (upload_info_len < MTP_OBJECT_INFO_FIXED_SIZE + 1u) return false;
  uint32_t storage_id;
  uint16_t object_format;
  uint32_t object_size;
  uint32_t parent;
  memcpy(&storage_id, upload_info + 0, sizeof(storage_id));
  memcpy(&object_format, upload_info + 4, sizeof(object_format));
  memcpy(&object_size, upload_info + 8, sizeof(object_size));
  memcpy(&parent, upload_info + 38, sizeof(parent));
  if (storage_id != MTP_STORAGE_ID || parent != 0 || object_format == MTP_OBJ_FORMAT_ASSOCIATION) {
    return false;
  }

  uint32_t pos = MTP_OBJECT_INFO_FIXED_SIZE;
  const uint8_t chars = upload_info[pos++];
  if (chars < 2 || chars > MTP_UPLOAD_NAME_MAX + 1u ||
      pos + (uint32_t)chars * 2u > upload_info_len) return false;
  uint8_t filename[MTP_UPLOAD_NAME_MAX];
  uint32_t filename_len = chars - 1u;
  for (uint32_t i = 0; i < filename_len; i++) {
    const uint16_t ch = (uint16_t)upload_info[pos + i * 2u] |
                        ((uint16_t)upload_info[pos + i * 2u + 1u] << 8);
    if (ch > 0x7Fu || !( (ch >= 'A' && ch <= 'Z') ||
        (ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9') ||
        ch == '_' || ch == '-' || ch == '.' || ch == ' ')) return false;
    filename[i] = (uint8_t)ch;
  }
  if ((filename_len == 1 && filename[0] == '.') ||
      (filename_len == 2 && filename[0] == '.' && filename[1] == '.') ||
      (filename_len >= 5 && memcmp(filename, ".mtp-", 5) == 0)) return false;
  pos += (uint32_t)chars * 2u;

  // Skip date-created, date-modified and keywords strings, validating bounds.
  for (unsigned field = 0; field < 3; field++) {
    if (pos >= upload_info_len) return false;
    const uint8_t count = upload_info[pos++];
    const uint32_t bytes = (uint32_t)count * 2u;
    if (pos + bytes > upload_info_len) return false;
    pos += bytes;
  }
  if (pos != upload_info_len) return false;
  if (!xcore_mtp_begin_upload(filename, filename_len, object_size)) return false;
  upload_expected_size = object_size;
  upload_info_valid = true;
  send_object_info_params[0] = MTP_STORAGE_ID;
  send_object_info_params[1] = 0;
  send_object_info_params[2] = xcore_mtp_find_object(filename, filename_len);
  if (send_object_info_params[2] == 0) {
    xcore_mtp_abort_upload();
    upload_info_valid = false;
    return false;
  }
  return true;
}

int32_t tud_mtp_command_received_cb(tud_mtp_cb_data_t *cb_data) {
  return dispatch_operation(cb_data);
}

int32_t tud_mtp_data_xfer_cb(tud_mtp_cb_data_t *cb_data) {
  const uint16_t operation = cb_data->command_container->header.code;
  if (operation == MTP_OP_SEND_OBJECT_INFO) {
    const uint32_t count = cb_data->io_container.payload_bytes;
    if (count > sizeof(upload_info) - upload_info_len) return -1;
    memcpy(upload_info + upload_info_len, cb_data->io_container.payload, count);
    upload_info_len += count;
    return 0;
  }
  if (operation == MTP_OP_SEND_OBJECT && upload_data_active) {
    const uint32_t payload_len = cb_data->io_container.payload_bytes;
    const uint32_t offset = cb_data->total_xferred_bytes -
        sizeof(mtp_container_header_t) - payload_len;
    return xcore_mtp_write_upload_chunk(offset, cb_data->io_container.payload, payload_len) ? 0 : -1;
  }
  const int32_t result = dispatch_operation(cb_data);
  return result < 0 ? result : 0;
}

int32_t tud_mtp_data_complete_cb(tud_mtp_cb_data_t *cb_data) {
  mtp_container_info_t *response = &cb_data->io_container;
  const uint16_t operation = cb_data->command_container->header.code;
  bool success = cb_data->xfer_result == XFER_RESULT_SUCCESS;

  if (operation == MTP_OP_SEND_OBJECT_INFO) {
    success = success && parse_upload_object_info();
    response->header->len = sizeof(mtp_container_header_t);
    if (success) {
      (void) mtp_container_add_uint32(response, send_object_info_params[0]);
      (void) mtp_container_add_uint32(response, send_object_info_params[1]);
      (void) mtp_container_add_uint32(response, send_object_info_params[2]);
    }
    response->header->code = success ? MTP_RESP_OK : MTP_RESP_INVALID_DATASET;
  } else if (operation == MTP_OP_SEND_OBJECT) {
    success = success && upload_data_active &&
        cb_data->total_xferred_bytes - sizeof(mtp_container_header_t) == upload_expected_size &&
        xcore_mtp_finish_upload();
    upload_data_active = false;
    upload_info_valid = false;
    response->header->len = sizeof(mtp_container_header_t);
    response->header->code = success ? MTP_RESP_OK : MTP_RESP_GENERAL_ERROR;
    if (!success) xcore_mtp_abort_upload();
  } else {
    response->header->len = sizeof(mtp_container_header_t);
    if (active_partial_object) {
      (void) mtp_container_add_uint32(response, active_object_length);
    }
    response->header->code = success ? MTP_RESP_OK : MTP_RESP_GENERAL_ERROR;
  }
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
