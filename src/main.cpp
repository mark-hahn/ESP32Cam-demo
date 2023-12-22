// https://github.com/alanesq/ESP32Cam-demo

#include <Arduino.h>
#include "esp_camera.h" 
// #include <esp_task_wdt.h>  
#include "esp32/rom/rtc.h"
#include <WiFi.h>
#include <WebServer.h>
#include "soc/soc.h"           // brownout
#include "soc/rtc_cntl_reg.h"  // brownout

#define SSID_NAME "hahn-fi"
#define SSID_PASWORD "90-NBVcvbasd"

#define SERIAL_SPEED 921600

bool initialiseCamera(bool); 
bool cameraImageSettings();
void changeResolution();
void handleNotFound();
bool handleJPG();

// ------
//  -SETTINGS
// ------

//  #define WDT_TIMEOUT 10
int cameraImageExposure = 0;  // Camera exposure (0 - 1200)  If gain and exposure both set to zero then auto adjust is enabled
int cameraImageGain = 0;  // Image gain (0 - 30)
int cameraImageBrightness = 0;  // Image brightness (-2 to +2)
const int camChangeDelay = 200;  // delay when deinit camera executed
#define CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM  32  // power to camera (on/off)
#define RESET_GPIO_NUM  -1  // -1 = not used
#define XCLK_GPIO_NUM  0
#define SIOD_GPIO_NUM  26  // i2c sda
#define SIOC_GPIO_NUM  27  // i2c scl
#define Y9_GPIO_NUM  35
#define Y8_GPIO_NUM  34
#define Y7_GPIO_NUM  39
#define Y6_GPIO_NUM  36
#define Y5_GPIO_NUM  21
#define Y4_GPIO_NUM  19
#define Y3_GPIO_NUM  18
#define Y2_GPIO_NUM  5
#define VSYNC_GPIO_NUM  25  // vsync_pin
#define HREF_GPIO_NUM  23  // href_pin
#define PCLK_GPIO_NUM  22  // pixel_clock_pin
camera_config_t config;  // camera settings

WebServer server(80);  // serve web pages on port 80

enum rst_reason {
  REASON_DEFAULT_RST      = 0,    /* normal startup by power on */
  REASON_WDT_RST          = 1,    /* hardware watch dog reset */
  REASON_EXCEPTION_RST    = 2,    /* exception reset, GPIO status won’t change */
  REASON_SOFT_WDT_RST     = 3,    /* software watch dog reset, GPIO status won’t change */
  REASON_SOFT_RESTART     = 4,    /* software restart ,system_restart , GPIO status won’t change */
  REASON_DEEP_SLEEP_AWAKE = 5,    /* wake up from deep-sleep */
  REASON_EXT_SYS_RST      = 6     /* external system reset */
};

void verbose_print_reset_reason(int reason) {
  if(reason < 2) return;
  switch ( reason) {
    case 1  : Serial.println ("Vbat power on reset");break;
    case 3  : Serial.println ("Software reset digital core");break;
    case 4  : Serial.println ("Legacy watch dog reset digital core");break;
    case 5  : Serial.println ("Deep Sleep reset digital core");break;
    case 6  : Serial.println ("Reset by SLC module, reset digital core");break;
    case 7  : Serial.println ("Timer Group0 Watch dog reset digital core");break;
    case 8  : Serial.println ("Timer Group1 Watch dog reset digital core");break;
    case 9  : Serial.println ("RTC Watch dog Reset digital core");break;
    case 10 : Serial.println ("Instrusion tested to reset CPU");break;
    case 11 : Serial.println ("Time Group reset CPU");break;
    case 12 : Serial.println ("Software reset CPU");break;
    case 13 : Serial.println ("RTC Watch dog Reset CPU");break;
    case 14 : Serial.println ("for APP CPU, reseted by PRO CPU");break;
    case 15 : Serial.println ("Reset when the vdd voltage is not stable");break;
    case 16 : Serial.println ("RTC Watch dog reset digital core and rtc module");break;
    default : Serial.println ("NO_MEAN");
  }
}

bool initialiseCamera(bool reset) {

// set the camera parameters in 'config'
if (reset) {
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;  // XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
  config.pixel_format = PIXFORMAT_JPEG;  // colour jpg format
  config.frame_size = FRAMESIZE_SXGA;  // Image sizes: 160x120 (QQVGA), 128x160 (QQVGA2), 176x144 (QCIF), 240x176 (HQVGA), 320x240 (QVGA),
  //  400x296 (CIF), 640x480 (VGA, default), 800x600 (SVGA), 1024x768 (XGA), 1280x1024 (SXGA),
  //  1600x1200 (UXGA)
  config.jpeg_quality = 10;  // 0-63 lower number means higher quality (can cause failed image capture if set too low at higher resolutions)
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  //config.fb_location = CAMERA_FB_IN_PSRAM;  // store the captured frame in PSRAM
  config.fb_count = 1;  // if more than one, i2s runs in continuous mode. Use only with JPEG
} // if reset

  esp_err_t camerr = esp_camera_init(&config);  // initialise the camera
  if (camerr != ESP_OK) {
  Serial.printf("ERROR: Camera init failed with error 0x%x", camerr);
  }

  cameraImageSettings();  // apply the camera image settings

  return (camerr == ESP_OK);
}


// -------
//  -Change camera image settings
// -------
// Adjust image properties (brightness etc.)
// Defaults to auto adjustments if exposure and gain are both set to zero

bool cameraImageSettings() {
  Serial.println("Applying camera settings");

  sensor_t *s = esp_camera_sensor_get();
  // something to try?:  if (s->id.PID == OV3660_PID)
  if (s == NULL) {
  Serial.println("Error: problem reading camera sensor settings");
  return 0;
  }

  // if both set to zero enable auto adjust
  if (cameraImageExposure == 0 && cameraImageGain == 0) {
  // enable auto adjust
  s->set_gain_ctrl(s, 1);  // auto gain on
  s->set_exposure_ctrl(s, 1);  // auto exposure on 
  s->set_awb_gain(s, 1);  // Auto White Balance enable (0 or 1)
  s->set_brightness(s, cameraImageBrightness);  // (-2 to 2) - set brightness
  } else {
  // Apply manual settings
  s->set_gain_ctrl(s, 0);  // auto gain off
  s->set_awb_gain(s, 1);  // Auto White Balance enable (0 or 1)
  s->set_exposure_ctrl(s, 0);  // auto exposure off
  s->set_brightness(s, cameraImageBrightness);  // (-2 to 2) - set brightness
  s->set_agc_gain(s, cameraImageGain);  // set gain manually (0 - 30)
  s->set_aec_value(s, cameraImageExposure);  // set exposure manually  (0-1200)
  }
  return 1;
}  // cameraImageSettings

// -------
//  send standard html header (i.e. start of web page)
// -------
void sendHeader(WiFiClient &client, char* hTitle) {
  // Start page
  client.write("HTTP/1.1 200 OK\r\n");
  client.write("Content-Type: text/html\r\n");
  client.write("Connection: close\r\n");
  client.write("\r\n");
  client.write("<!DOCTYPE HTML><html lang='en'>\n");
  // HTML / CSS
  client.printf(R"=====(
  <head>
  <meta name='viewport' content='width=device-width, initial-scale=1.0'>
  <title>%s</title>
  <style>
  body {
  color: black;
  background-color: #FFFF00;
  text-align: center;
  }
  input {
  background-color: #FF9900;
  border: 2px #FF9900;
  color: blue;
  padding: 3px 6px;
  text-align: center;
  text-decoration: none;
  display: inline-block;
  font-size: 16px;
  cursor: pointer;
  border-radius: 7px;
  }
  input:hover {
  background-color: #FF4400;
  }
  </style>
  </head>
  <body>
  <h1 style='color:red;'>%s</H1>
  )=====", hTitle, hTitle);
}

void sendFooter(WiFiClient &client) {
  client.write("</body></html>\n");
  delay(3);
  client.stop();
}

void resetCamera(bool type = 0) {
  if (type == 1) {
  // power cycle the camera module (handy if camera stops responding)
  digitalWrite(PWDN_GPIO_NUM, HIGH);  // turn power off to camera module
  delay(300);
  digitalWrite(PWDN_GPIO_NUM, LOW);
  delay(300);
  initialiseCamera(1);
  } else {
  // reset via software (handy if you wish to change resolution or image type etc. - see test procedure)
  esp_camera_deinit();
  delay(camChangeDelay);
  initialiseCamera(1);
  }
}

void handleNotFound() {
 String tReply;
 Serial.print("Invalid page requested");
 tReply = "File Not Found\n\n";
 tReply += "URI: ";
 tReply += server.uri();
 tReply += "\nMethod: ";
 tReply += ( server.method() == HTTP_GET ) ? "GET" : "POST";
 tReply += "\nArguments: ";
 tReply += server.args();
 tReply += "\n";

 for ( uint8_t i = 0; i < server.args(); i++ ) {
  tReply += " " + server.argName ( i ) + ": " + server.arg ( i ) + "\n";
 }

 server.send ( 404, "text/plain", tReply );
 tReply = "";  // clear variable

}  // handleNotFound

// -------
//  -capture jpg image and send  i.e. http://x.x.x.x/jpg
// -------

bool handleJPG() {

  WiFiClient client = server.client();  // open link with client
  char buf[32];

  // capture the jpg image from camera
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  // // there is a bug where this buffer can be from previous capture so as workaround it is discarded and captured again
  // esp_camera_fb_return(fb); // dispose the buffered image
  // fb = NULL;
  // fb = esp_camera_fb_get(); // get fresh image
  if (!fb) {
    Serial.println("Error: failed to capture image");
    return 0;
  }
  else Serial.println("Image captured");

  // html to send a jpg
  const char HEADER[] = "HTTP/1.1 200 OK\r\nAccess-Control-Allow-Origin: *\r\n";
  const char CTNTTYPE[] = "Content-Type: image/jpeg\r\nContent-Length: ";
  const int hdrLen = strlen(HEADER);
  const int cntLen = strlen(CTNTTYPE);
  client.write(HEADER, hdrLen);
  client.write(CTNTTYPE, cntLen);
  sprintf( buf, "%d\r\n\r\n", fb->len);  // put text size in to 'buf' char array and send
  client.write(buf, strlen(buf));

  // send the captured jpg data
  client.write((char *)fb->buf, fb->len);
  delay(3);
  client.stop();

  esp_camera_fb_return(fb);  // return camera frame buffer
  return 1;
}  // handleJPG

void setup() {
  Serial.begin(SERIAL_SPEED); 
  Serial.println();
  Serial.printf("Setup");

  verbose_print_reset_reason(rtc_get_reset_reason(0));
  verbose_print_reset_reason(rtc_get_reset_reason(1));

//  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // Turn-off the 'brownout detector'

 // Connect to wifi
  Serial.print("\nConnecting to ");
  Serial.print(SSID_NAME);
  Serial.print("\n  ");

  WiFi.begin(SSID_NAME, SSID_PASWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("\nWiFi connected, ");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();

  server.on("/jpg", handleJPG);  // capture image and send as jpg
  server.onNotFound(handleNotFound);  // invalid url requested

 // set up camera
  if (initialiseCamera(1)) {
    Serial.println("initializeCamera OK");
  }
  else {
    Serial.println("initialiseCamera failed");
  }
  // Serial.println("Configuring watchdog timer");
  // esp_task_wdt_init(WDT_TIMEOUT, true);
  // esp_task_wdt_add(NULL);  //add current thread to WDT watch  

 // startup complete
  Serial.println("\nReady for jpg page load ...");
  delay(200);
}  // setup

void loop() {
 server.handleClient();  // handle any incoming web page requests
}
