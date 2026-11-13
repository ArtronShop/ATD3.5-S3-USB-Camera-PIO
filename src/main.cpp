// USB (UVC) camera viewer with FPS counter for ATD3.5-S3
//  - USB host stack + UVC class driver (MJPEG) from the Arduino-ESP32 prebuilt libs
//  - JPEG decode with esp_jpeg (TJpgDec), shown with Display.drawBitmap()
//  - FPS shown at the top-right corner of the screen
#include <Arduino.h>
#include <ATD3.5-S3.h>
#include "esp_heap_caps.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"
#include "jpeg_decoder.h"   // esp_jpeg (TJpgDec) - only used when the image must be scaled down
#include "esp_jpeg_dec.h"   // esp_new_jpeg - fast decoder (full size)

// Hybrid-compile (custom_sdkconfig) leaves "-Wl,--wrap=log_printf" in the link flags
// but not the wrapper itself, so provide it here.
extern "C" int __wrap_log_printf(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  int n = vprintf(fmt, ap);
  va_end(ap);
  return n;
}

// ---- Camera settings (must be a mode your camera supports as MJPEG) ----
// The mode is picked automatically from what the camera offers (MJPEG only):
// the largest frame not bigger than MAX_CAM_W x MAX_CAM_H, with FPS closest to TARGET_FPS.
#define MAX_CAM_W   480
#define MAX_CAM_H   320
#define TARGET_FPS  30
#define JPEG_MAX_SIZE (128 * 1024)

static unsigned camW = 0, camH = 0;
static float camFps = 0;
static esp_jpeg_image_scale_t jpegScale = JPEG_IMAGE_SCALE_0;
static int outW = 0, outH = 0, imgX = 0, imgY = 0;
#define OUT_WIDTH   outW
#define OUT_HEIGHT  outH
#define IMG_X imgX
#define IMG_Y imgY
#define JPEG_SCALE jpegScale

static uint8_t  *jpegBuf = nullptr;   // latest MJPEG frame from camera
static size_t    jpegLen = 0;
static volatile bool frameReady = false;
static uint16_t *outBufs[2] = {nullptr, nullptr}; // decoded RGB565 (big-endian for the LCD), double buffered
static QueueHandle_t freeQ, drawQ;    // buffer indices: free for decoding / ready to draw
static volatile uint32_t tDecode = 0, tDraw = 0, tShown = 0; // cumulative µs / frames
static SemaphoreHandle_t jpegMutex;
static volatile bool camConnected = false;

// ---------------- Tiny 5x7 font for the FPS overlay ----------------
struct Glyph { char c; uint8_t col[5]; };
static const Glyph FONT[] = {
  {'0', {0x3E,0x51,0x49,0x45,0x3E}}, {'1', {0x00,0x42,0x7F,0x40,0x00}},
  {'2', {0x42,0x61,0x51,0x49,0x46}}, {'3', {0x21,0x41,0x45,0x4B,0x31}},
  {'4', {0x18,0x14,0x12,0x7F,0x10}}, {'5', {0x27,0x45,0x45,0x45,0x39}},
  {'6', {0x3C,0x4A,0x49,0x49,0x30}}, {'7', {0x01,0x71,0x09,0x05,0x03}},
  {'8', {0x36,0x49,0x49,0x49,0x36}}, {'9', {0x06,0x49,0x49,0x29,0x1E}},
  {'.', {0x00,0x60,0x60,0x00,0x00}}, {' ', {0x00,0x00,0x00,0x00,0x00}},
  {'F', {0x7F,0x09,0x09,0x09,0x01}}, {'P', {0x7F,0x09,0x09,0x09,0x06}},
  {'S', {0x46,0x49,0x49,0x49,0x31}},
};

static const uint8_t *findGlyph(char c) {
  for (auto &g : FONT) if (g.c == c) return g.col;
  return FONT[11].col; // space
}

// Draw text at the top-right of the screen (scale = pixel size of each font dot)
static void drawTextTopRight(const char *text, uint8_t scale, uint16_t fg, uint16_t bg) {
  static uint16_t lineBuf[LCD_WIDTH * 7 * 4];
  const int charW = 6 * scale, h = 7 * scale + 2 * scale;
  int len = strlen(text);
  int w = len * charW + 2 * scale;
  if (w > LCD_WIDTH) return;
  uint16_t fgS = __builtin_bswap16(fg), bgS = __builtin_bswap16(bg);
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      uint16_t px = bgS;
      int gy = y / scale - 1, cx = x - scale;
      if (gy >= 0 && gy < 7 && cx >= 0) {
        int ci = cx / charW, gx = (cx % charW) / scale;
        if (ci < len && gx < 5 && (findGlyph(text[ci])[gx] >> gy & 1)) px = fgS;
      }
      lineBuf[y * w + x] = px;
    }
  }
  Display.drawBitmap(LCD_WIDTH - w, 0, LCD_WIDTH - 1, h - 1, lineBuf);
}

// ---------------- USB host / UVC ----------------
static void usbLibTask(void *) {
  while (true) {
    uint32_t flags;
    usb_host_lib_handle_events(portMAX_DELAY, &flags);
    if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) usb_host_device_free_all();
  }
}

static volatile uint32_t statRx = 0, statSkip = 0, statBig = 0, statDecErr = 0;

static bool frameCallback(const uvc_host_frame_t *frame, void *) {
  statRx++;
  if (frame->data_len > JPEG_MAX_SIZE) statBig++;
  if (frameReady) statSkip++;
  if (!frameReady && frame->data_len > 0 && frame->data_len <= JPEG_MAX_SIZE) {
    if (xSemaphoreTake(jpegMutex, 0) == pdTRUE) {
      memcpy(jpegBuf, frame->data, frame->data_len);
      jpegLen = frame->data_len;
      frameReady = true;
      xSemaphoreGive(jpegMutex);
    }
  }
  return true; // frame already copied -> ownership goes back to the driver immediately
}

static void streamEventCallback(const uvc_host_stream_event_data_t *ev, void *) {
  if (ev->type == UVC_HOST_DEVICE_DISCONNECTED) {
    camConnected = false;
    uvc_host_stream_close(ev->device_disconnected.stream_hdl);
    Serial.println("Camera disconnected");
  } else if (ev->type == UVC_HOST_TRANSFER_ERROR) {
    Serial.printf("USB transfer error: %d\n", ev->transfer_error.error);
  }
}

// Read the camera's mode list and choose a MJPEG mode. Returns false if none.
static bool pickMode(uint8_t devAddr, uint8_t streamIdx) {
  static uvc_host_frame_info_t list[48];
  size_t n = sizeof(list) / sizeof(list[0]);
  esp_err_t err = uvc_host_get_frame_list(devAddr, streamIdx, &list, &n);
  if (err != ESP_OK) {
    Serial.printf("uvc_host_get_frame_list failed: %s\n", esp_err_to_name(err));
    return false;
  }
  Serial.printf("Camera offers %u modes:\n", (unsigned)n);
  long bestArea = 0;
  for (size_t i = 0; i < n; i++) {
    const uvc_host_frame_info_t &f = list[i];
    const char *fmt = f.format == UVC_VS_FORMAT_MJPEG ? "MJPEG" : f.format == UVC_VS_FORMAT_YUY2 ? "YUY2" :
                      f.format == UVC_VS_FORMAT_H264 ? "H264" : f.format == UVC_VS_FORMAT_NV12 ? "NV12" : "other";
    // choose the FPS closest to TARGET_FPS (intervals are in 100 ns units)
    const uint32_t want = 10000000UL / TARGET_FPS;
    uint32_t iv = f.default_interval;
    if (f.interval_type == 0) {
      iv = want < f.interval_min ? f.interval_min : (want > f.interval_max ? f.interval_max : want);
    } else {
      long bestD = 0x7FFFFFFF;
      for (int k = 0; k < f.interval_type && k < CONFIG_UVC_INTERVAL_ARRAY_SIZE; k++) {
        long d = labs((long)f.interval[k] - (long)want);
        if (d < bestD) { bestD = d; iv = f.interval[k]; }
      }
    }
    float fps = iv ? 10000000.0f / iv : 0;
    Serial.printf("  %-5s %ux%u @ %.1f fps\n", fmt, f.h_res, f.v_res, fps);
    if (f.format != UVC_VS_FORMAT_MJPEG || f.h_res > MAX_CAM_W || f.v_res > MAX_CAM_H) continue;
    long area = (long)f.h_res * f.v_res;
    if (area > bestArea || (area == bestArea && fps > camFps)) {
      bestArea = area; camW = f.h_res; camH = f.v_res; camFps = fps;
    }
  }
  if (!bestArea) {
    Serial.println("No MJPEG mode <= MAX_CAM_W x MAX_CAM_H. Raise MAX_CAM_W/H or use a MJPEG camera.");
    return false;
  }
  // smallest decoder scale so the image fits on the screen
  int s = 0;
  while (s < 3 && ((camW >> s) > LCD_WIDTH || (camH >> s) > LCD_HEIGHT)) s++;
  jpegScale = (esp_jpeg_image_scale_t)s;
  outW = camW >> s;
  outH = camH >> s;
  imgX = (LCD_WIDTH - outW) / 2;
  imgY = (LCD_HEIGHT - outH) / 2;
  Serial.printf("Using MJPEG %ux%u @ %.1f fps (scale 1/%d -> %dx%d)\n", camW, camH, camFps, 1 << s, outW, outH);
  return true;
}

static void openCamera() {
  const uvc_host_stream_config_t cfg = {
    .event_cb = streamEventCallback,
    .frame_cb = frameCallback,
    .user_ctx = nullptr,
    .usb = { .dev_addr = UVC_HOST_ANY_DEV_ADDR, .vid = UVC_HOST_ANY_VID,
             .pid = UVC_HOST_ANY_PID, .uvc_stream_index = 0 },
    .vs_format = { .h_res = camW, .v_res = camH, .fps = camFps,
                   .format = UVC_VS_FORMAT_MJPEG },
    .advanced = { .number_of_frame_buffers = 3, .frame_size = 0,
                  .frame_heap_caps = MALLOC_CAP_SPIRAM, .number_of_urbs = 3,
                  .urb_size = 0, .user_frame_buffers = nullptr },
  };
  uvc_host_stream_hdl_t hdl = nullptr;
  esp_err_t err = uvc_host_stream_open(&cfg, 1000, &hdl);
  if (err != ESP_OK) {
    Serial.printf("uvc_host_stream_open failed: %s\n", esp_err_to_name(err));
    return;
  }
  err = uvc_host_stream_start(hdl);
  if (err != ESP_OK) {
    Serial.printf("uvc_host_stream_start failed: %s\n", esp_err_to_name(err));
    uvc_host_stream_close(hdl);
    return;
  }
  camConnected = true;
  Serial.println("Camera streaming");
}

static volatile bool newDevice = false;
static uint8_t newDevAddr = 0, newStreamIdx = 0;
static void driverEventCallback(const uvc_host_driver_event_data_t *ev, void *) {
  if (ev->type == UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED) {
    newDevAddr = ev->device_connected.dev_addr;
    newStreamIdx = ev->device_connected.uvc_stream_index;
    newDevice = true;
  }
}

// Draws decoded frames + FPS text. The only task that touches the LCD while streaming.
static void drawTask(void *) {
  uint32_t fpsCount = 0, fpsT0 = millis();
  float fps = 0;
  while (true) {
    int idx;
    if (xQueueReceive(drawQ, &idx, portMAX_DELAY) != pdTRUE) continue;
    uint32_t t1 = micros();
    // Draw in strips and sleep 1 tick between them: drawBitmap() busy-waits on SPI, so one
    // long call would starve IDLE0 on this core and trigger the task watchdog.
    const int STRIP = 32;
    for (int y = 0; y < OUT_HEIGHT; y += STRIP) {
      int y2 = min(y + STRIP, (int)OUT_HEIGHT) - 1;
      Display.drawBitmap(IMG_X, IMG_Y + y, IMG_X + OUT_WIDTH - 1, IMG_Y + y2, outBufs[idx] + (size_t)y * OUT_WIDTH);
      vTaskDelay(1);
    }
    tDraw += micros() - t1;
    tShown++;
    xQueueSend(freeQ, &idx, portMAX_DELAY); // buffer can be decoded into again

    // FPS: update twice a second
    fpsCount++;
    uint32_t now = millis();
    if (now - fpsT0 >= 500) {
      fps = fpsCount * 1000.0f / (now - fpsT0);
      fpsCount = 0;
      fpsT0 = now;
    }
    char txt[16];
    snprintf(txt, sizeof(txt), "FPS %.1f", fps);
    drawTextTopRight(txt, 3, 0xFFE0, 0x0000); // yellow on black
  }
}

// Decode jpegBuf into outBufs[idx]. Returns true on success.
static bool decodeFrame(int idx) {
  uint32_t t0 = micros();
  bool ok = false;
  if (jpegScale == JPEG_IMAGE_SCALE_0) {
    jpeg_dec_config_t cfg = DEFAULT_JPEG_DEC_CONFIG();
    cfg.output_type = JPEG_PIXEL_FORMAT_RGB565_BE; // LCD expects big-endian RGB565
    jpeg_dec_handle_t dec = nullptr;
    if (jpeg_dec_open(&cfg, &dec) == JPEG_ERR_OK) {
      jpeg_dec_io_t io = {};
      io.inbuf = jpegBuf;
      io.inbuf_len = (int)jpegLen;
      io.outbuf = (uint8_t *)outBufs[idx];
      jpeg_dec_header_info_t info;
      ok = jpeg_dec_parse_header(dec, &io, &info) == JPEG_ERR_OK &&
           info.width == outW && info.height == outH &&
           jpeg_dec_process(dec, &io) == JPEG_ERR_OK;
      jpeg_dec_close(dec);
    }
  } else { // needs down-scaling: use the TJpgDec based decoder
    esp_jpeg_image_cfg_t jc = {};
    jc.indata = jpegBuf;
    jc.indata_size = jpegLen;
    jc.outbuf = (uint8_t *)outBufs[idx];
    jc.outbuf_size = OUT_WIDTH * OUT_HEIGHT * 2;
    jc.out_format = JPEG_IMAGE_FORMAT_RGB565;
    jc.out_scale = JPEG_SCALE;
    jc.flags.swap_color_bytes = 1;
    esp_jpeg_image_output_t out;
    ok = (esp_jpeg_decode(&jc, &out) == ESP_OK);
  }
  tDecode += micros() - t0;
  return ok;
}

void setup() {
  Serial.begin(115200);

  Display.begin(0, 40E6);
  Display.fillScreen(0x0000);

  jpegMutex = xSemaphoreCreateMutex();
  jpegBuf = (uint8_t *)heap_caps_aligned_alloc(16, JPEG_MAX_SIZE, MALLOC_CAP_SPIRAM);
  for (auto &b : outBufs) b = (uint16_t *)heap_caps_aligned_alloc(16, LCD_WIDTH * LCD_HEIGHT * 2, MALLOC_CAP_SPIRAM);
  if (!jpegBuf || !outBufs[0] || !outBufs[1]) {
    Serial.println("Out of memory (PSRAM)");
    while (true) delay(1000);
  }
  freeQ = xQueueCreate(2, sizeof(int));
  drawQ = xQueueCreate(1, sizeof(int));
  for (int i = 0; i < 2; i++) xQueueSend(freeQ, &i, 0);
  // Draw task (SPI to LCD) runs on core 0 so it overlaps with decoding on core 1.
  xTaskCreatePinnedToCore(drawTask, "draw", 4096, nullptr, 5, nullptr, 0);

  // USB host + UVC driver
  const usb_host_config_t hostCfg = { .skip_phy_setup = false, .intr_flags = ESP_INTR_FLAG_LEVEL1 };
  ESP_ERROR_CHECK(usb_host_install(&hostCfg));
  xTaskCreatePinnedToCore(usbLibTask, "usb_lib", 4096, nullptr, 10, nullptr, 0);

  const uvc_host_driver_config_t drvCfg = {
    .driver_task_stack_size = 4096,
    .driver_task_priority = 11,
    .xCoreID = 0,
    .create_background_task = true,
    .event_cb = driverEventCallback,
    .user_ctx = nullptr,
  };
  ESP_ERROR_CHECK(uvc_host_install(&drvCfg));
  Serial.println("Waiting for USB camera...");
}

void loop() {
  static bool shownWait = false;

  static uint32_t statT = 0;
  if (camConnected && millis() - statT >= 1000) {
    statT = millis();
    unsigned n = tShown ? tShown : 1;
    Serial.printf("[stat] rx=%u skipped=%u too_big=%u decode_err=%u | shown=%u avg decode=%u ms draw=%u ms jpeg=%u B\n",
                  (unsigned)statRx, (unsigned)statSkip, (unsigned)statBig, (unsigned)statDecErr,
                  (unsigned)tShown, (unsigned)(tDecode / 1000 / n), (unsigned)(tDraw / 1000 / n), (unsigned)jpegLen);
  }

  if (newDevice) {
    newDevice = false;
    if (pickMode(newDevAddr, newStreamIdx)) openCamera();
  }

  if (!frameReady) {
    if (!camConnected && !shownWait) {
      shownWait = true;
      drawTextTopRight("FPS 0.0", 3, 0xFFFF, 0x0000);
    }
    delay(1);
    return;
  }
  shownWait = false;

  // Get a free output buffer (waits while the draw task is still using both)
  int idx;
  if (xQueueReceive(freeQ, &idx, portMAX_DELAY) != pdTRUE) return;

  // Decode the latest frame (camera callback may not overwrite it meanwhile)
  bool ok = false;
  if (xSemaphoreTake(jpegMutex, portMAX_DELAY) == pdTRUE) {
    ok = decodeFrame(idx);
    frameReady = false;
    xSemaphoreGive(jpegMutex);
  }
  if (!ok) {
    statDecErr++;
    xQueueSend(freeQ, &idx, 0);
    return;
  }
  xQueueSend(drawQ, &idx, portMAX_DELAY); // hand over to the draw task
}
