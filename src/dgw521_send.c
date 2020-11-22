#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdint.h>
#include <glib.h>
#include <glib-unix.h>
#include <modbus-rtu.h>

GQuark
dgw_error_quark()
{
  static GQuark error_quark = 0;
  if (error_quark == 0)
    error_quark =
      g_quark_from_static_string ("DGW-521-error-quark");
  return error_quark;
}

#define DGW_ERROR (dgw_error_quark())
enum {
  DGW_ERROR_OK = 0,
  DGW_ERROR_READ,
  DGW_ERROR_WRITE,
  DGW_ERROR_PARAMETER
};
  
typedef struct ModbusSource ModbusSource;
struct ModbusSource {
  GSource source;
  gpointer tag;
};

typedef struct AppContext AppContext;
struct AppContext
{
  gchar *device;
  guint speed;
  guint mb_addr;

  
  modbus_t *mb;

  uint16_t n_cmds;
  uint16_t *cmds;
  uint16_t *replies;
};


static void
app_init(AppContext *app)
{
  app->device = "/dev/ttyACM0";
  app->speed = 38400;
  app->mb_addr = 1;
  app->cmds = NULL;
  app->replies = NULL;
  app->n_cmds = 0;
}

static void
app_cleanup(AppContext* app)
{
  g_free(app->cmds);
  g_free(app->replies);
  if (app->mb) {
    modbus_close(app->mb);
    modbus_free(app->mb);
    app->mb = NULL;
  }
}

#define MB_ADDR_FW_LOW 480
#define MB_ADDR_FW_HIGH 481
#define MB_ADDR_MODNAME_LOW 482
#define MB_ADDR_MODNAME_HIGH 483
#define MB_ADDR_BUS_ADDR 484
#define MB_ADDR_SER_CONF 485
#define MB_ADDR_RESP_DELAY 487
#define MB_ADDR_WD_TIMEOUT 488
#define MB_ADDR_WD_COUNT 491
#define MB_ADDR_PROTO 256
#define MB_ADDR_WD_ENABLED 260
#define MB_ADDR_CMD_QUEUE 32
#define MB_ADDR_REPLY_QUEUE 0
#define MB_ADDR_CMD_READY 256




static gboolean
init_modbus(AppContext *app)
{
  app->mb = modbus_new_rtu(app->device, app->speed, 'E', 8, 1);
  if (!app->mb) {
    g_printerr("Failed to create Modbus context\n");
    return FALSE;
  }
  modbus_set_debug(app->mb, 0);
  modbus_set_slave(app->mb,app->mb_addr);
  if (modbus_connect(app->mb)) {
    g_printerr("Failed to connect\n");
    return FALSE;
  }
  
  return TRUE;
}


AppContext app;

const GOptionEntry app_options[] = {
  {"device", 'd', 0, G_OPTION_ARG_STRING,
   &app.device, "Serial device", "DEV"},
  {"speed", 's', 0, G_OPTION_ARG_INT,
   &app.speed, "Serial speed (bps)", "SPEED"},
  {"mb-addr", 0, 0, G_OPTION_ARG_INT,
   &app.mb_addr, "Modbus address of DGW-521", "ADDR"},
  {NULL}
};

static gboolean
send_cmd(modbus_t *mb, uint16_t *cmds, uint16_t *replies,
	 unsigned int len, GError **err)
{

  while(len > 0) {
    int block_len;
    if (len > 8) {
      block_len = 8;
    } else {
      block_len = len;
    }    
    int w = modbus_write_registers(mb, MB_ADDR_CMD_QUEUE, block_len, cmds);
    if (w <= 0) {
      g_set_error(err, DGW_ERROR, DGW_ERROR_WRITE, 
		  "Failed to write to command queue");
      return FALSE;
    }
    while(TRUE) {
      uint16_t ready;
      int s = modbus_read_registers(mb, MB_ADDR_CMD_READY, 1, &ready);
      if (s != 1) {
	g_set_error(err, DGW_ERROR, DGW_ERROR_WRITE, 
		    "Failed to read command done status");
	return FALSE;
      }
      if (ready == 0xff) break;
    }
    int r = modbus_read_registers(mb, MB_ADDR_REPLY_QUEUE, block_len, replies);
    if (r <= 0) {
      g_set_error(err, DGW_ERROR, DGW_ERROR_WRITE, 
		  "Failed to read replies");
      return FALSE;
    }
    len -= block_len;
    cmds += block_len;
    replies += block_len;
  }
  return TRUE;
}

#define MAX_CMDS 16
int
main(int argc, char **argv)
{
  GError *err = NULL;
  GOptionContext *opt_ctxt;
  app_init(&app);
  opt_ctxt = g_option_context_new (" - get setup info from DGW521");
  g_option_context_add_main_entries(opt_ctxt, app_options, NULL);
  if (!g_option_context_parse(opt_ctxt, &argc, &argv, &err)) {
    g_printerr("Failed to parse options: %s\n", err->message);
    app_cleanup(&app);
    return EXIT_FAILURE;
  }
  g_option_context_free(opt_ctxt);
  if (!init_modbus(&app)) {
    app_cleanup(&app);
    return EXIT_FAILURE;
  }


  app.n_cmds = argc - 1;
  app.cmds = g_new(uint16_t, app.n_cmds); 
  app.replies = g_new(uint16_t, app.n_cmds); 

  for (int c = 0; c < app.n_cmds; c++) {
    char *end;
    app.cmds[c] = strtoul(argv[c+1], &end, 16);
    if (end == argv[c+1] || *end != '\0') {
      g_printerr("Invalid command %s",argv[c+1]);
      return EXIT_FAILURE;
    }
  }
  for (int c = 0; c < app.n_cmds; c++) {
    printf("%04x\n", app.cmds[c], app.replies[c]);
  }
  
  if (!send_cmd(app.mb, app.cmds, app.replies, app.n_cmds, &err)) {
    g_printerr("Failed to send commands: %s\n", err->message);
    app_cleanup(&app);
    return EXIT_FAILURE;
  }
  
  for (int c = 0; c < app.n_cmds; c++) {
    printf("%04x => %04x\n", app.cmds[c], app.replies[c]);
  }
  
  app_cleanup(&app);
  return EXIT_SUCCESS;
}
