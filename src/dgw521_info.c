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

  gint set_addr;
  gchar *set_serial;
  gboolean watchdog_enable;
  gboolean watchdog_disable;
  gdouble watchdog_timeout;
  
  modbus_t *mb;
  GThread *mb_thread;
  GMutex mb_mutex;
  GCond mb_cond;
  gboolean mb_thread_running;
};


static void
app_init(AppContext *app)
{
  app->device = "/dev/ttyACM0";
  app->speed = 38400;
  app->mb_addr = 1;
  app->set_addr = -1;
  app->set_serial = NULL;
  app->mb = NULL;
  app->mb_thread_running = FALSE;
  g_mutex_init(&app->mb_mutex);
  g_cond_init(&app->mb_cond);
}

static void
app_cleanup(AppContext* app)
{
  if (app->mb) {
    modbus_close(app->mb);
    modbus_free(app->mb);
    app->mb = NULL;
  }
  g_free(app->set_serial);
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


#define MB_ADDR_RECORDS 1024
/* Don't read the full buffer since the oldest records risk being overwritten.*/
#define MAX_RECORDS 24

#if 0
static gpointer 
modbus_poll(gpointer data)
{
  uint16_t records[MAX_RECORDS*2];
  int r;
  uint16_t last_seq;
  AppContext *app = data;
  g_mutex_lock(&app->mb_mutex);
  app->mb_thread_running = TRUE;
  g_cond_signal(&app->mb_cond);
  g_mutex_unlock(&app->mb_mutex);
  g_debug("Thread running");
  r = modbus_read_input_registers(app->mb, MB_ADDR_SEQUENCE, 1, &last_seq);
  if (r == 1) {
    g_debug("Start: %d", last_seq);
  } else {
    g_printerr("Failed to read first sequence number");
  }
  while(app->mb_thread_running) {
    uint16_t seq;
    g_usleep(G_USEC_PER_SEC/10);
    r = modbus_read_input_registers(app->mb, MB_ADDR_SEQUENCE, 1, &seq);
    if (r == 1) {
      if (seq != last_seq) {
	uint16_t start;
	uint16_t end;
	uint16_t len;
	g_debug("Sequence: %d", seq);
	len = seq - last_seq;
	if (len > MAX_RECORDS) {
	  len = MAX_RECORDS;
	  g_printerr("Overrun");
	}
	end = seq + 1;
	start = (end - len) & 0x1f;
	end &= 0x1f;
	start *= 2;
	end *=2;
	g_debug("%d - %d",start, end);
	if (start < end) {
	  r = modbus_read_input_registers(app->mb,
					  MB_ADDR_RECORDS+start, end - start,
					  records);
	} else {
	  r = modbus_read_input_registers(app->mb, MB_ADDR_RECORDS+start, 
					  64 - start, records);
	  if (r > 0 && end > 0) {
	    r = modbus_read_input_registers(app->mb, MB_ADDR_RECORDS,
					    end, records+(64 - start));
	  }
	}
	if (r > 0) {
	  unsigned int i;
	  g_debug("Got %d records", len);
	  for (i = 0; i < len; i++) {
	    printf(" %04x %04x",records[i*2], records[i*2+1]);
	  }
	  fputc('\n', stdout);
	} else {
	  g_printerr("Failed to read records");
	}
	
	  
	last_seq = seq;
      }
    } else {
      g_printerr("Failed to read sequece number");
    }
  }
  g_debug("Thread exiting");
  return NULL;
}

static void
stop_mb_thread(AppContext *app)
{
  if (app->mb_thread) {
    app->mb_thread_running = FALSE;  // No need to worry about concurrency
    g_thread_join(app->mb_thread);
    app->mb_thread = NULL;
  }
}
#endif

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
  {"set-addr", 0, 0, G_OPTION_ARG_INT,
   &app.set_addr,  "Set new modbus address of DGW-521", "ADDR"},
  {"set-serial", 0, 0, G_OPTION_ARG_STRING,
   &app.set_serial,  "Set serial configuration of DGW-521", "BAUD,[N|O|E,[1|2]]"},
  {"set-watchdog-enabled", 0, 0, G_OPTION_ARG_NONE,
   &app.watchdog_enable,  "Enable watchdog", NULL},
  {"set-watchdog-disabled", 0, 0, G_OPTION_ARG_NONE,
   &app.watchdog_disable,  "Enable watchdog", NULL},
  {"set-watchdog-timeout", 0, 0, G_OPTION_ARG_DOUBLE,
   &app.watchdog_timeout,  "Watchdog timeout in seconds", NULL},
  {NULL}
};

static gboolean
write_mb_addr(modbus_t *mb, gint addr, GError **err)
{
  if (addr < 1 || addr > 247) {
    g_set_error(err, DGW_ERROR, DGW_ERROR_PARAMETER, 
		"Invalid Modbus address");
    return FALSE;
  }
  int w = modbus_write_register(mb,
				MB_ADDR_BUS_ADDR,
				addr);
    if (w <= 0) {
      g_set_error(err, DGW_ERROR, DGW_ERROR_WRITE, 
		  "Failed to write Modbus address setting");
      return FALSE;
    }
    modbus_flush(mb);
    return TRUE;
}

static gboolean
write_serial_settings(modbus_t *mb, const gchar *serstr, GError **err)
{
  char *end;
  int w;
  if (serstr) {
    int baud;
    uint16_t ser_conf = 0x00;
    printf("Setting serial parameters\n");
    baud = strtoul(serstr, &end, 10);
    if (serstr == end) {
     g_set_error(err, DGW_ERROR, DGW_ERROR_PARAMETER, 
		 "Unparseable baud rate");
     return FALSE;
    }
    serstr = end;
    switch(baud) {
    case 1200:
      ser_conf = 3;
      break;
    case 2400:
      ser_conf = 4;
      break;
    case 4800:
      ser_conf = 5;
      break;
    case 9600:
      ser_conf = 6;
      break;
    case 19200:
      ser_conf = 7;
      break;
    case 38400:
      ser_conf = 8;
      break;
    case 57600:
      ser_conf = 9;
      break;
    case 115200:
      ser_conf = 10;
      break;
    default:
      g_set_error(err, DGW_ERROR, DGW_ERROR_PARAMETER, 
		 "Invalid baud rate");
      return FALSE;
    }
    if (*serstr != '\0') {
      if (*serstr != ',') {
	g_set_error(err, DGW_ERROR, DGW_ERROR_PARAMETER, 
		    "Expected comma after baud rate");
	return FALSE;
      }
      serstr++;
      switch(*serstr) {
      case 'N':
      case 'n':
	break;
      case 'O':
      case 'o':
	ser_conf |= 0xc0;
	break;
      case 'E':
      case 'e':
	ser_conf |= 0x80;
	break;
      default:
	g_set_error(err, DGW_ERROR, DGW_ERROR_PARAMETER, 
		    "Parity must be 'O', 'E' or 'N'");
	return FALSE;
      }
    }
      
    w = modbus_write_register(mb,
			      MB_ADDR_SER_CONF,
			      ser_conf);
    if (w <= 0) {
      g_set_error(err, DGW_ERROR, DGW_ERROR_WRITE, "Failed to write serial settings");
      return FALSE;
    }
    modbus_flush(mb);
  }
  return TRUE;
}

static gboolean
write_wd_enable(modbus_t *mb, gboolean enable, GError **err)
{
  uint8_t e = enable;
  int w = modbus_write_bits(mb, MB_ADDR_WD_ENABLED, 1, &e);
  if (w <= 0) {
    g_set_error(err, DGW_ERROR, DGW_ERROR_WRITE, 
		"Failed to enable/disable watchdog");
    return FALSE;
  }
  modbus_flush(mb);
  return TRUE;
}

static gboolean
write_watchdog_timeout(modbus_t *mb, gdouble timeout, GError **err)
{
  int timeout_int = timeout*10;
  if (timeout_int < 1 || timeout_int > 255) {
    g_set_error(err, DGW_ERROR, DGW_ERROR_PARAMETER, 
		"Invalid timeout value");
    return FALSE;
  }
  int w = modbus_write_register(mb,
				MB_ADDR_WD_TIMEOUT,
				timeout_int);
    if (w <= 0) {
      g_set_error(err, DGW_ERROR, DGW_ERROR_WRITE, 
		  "Failed to write watchdog timeout setting");
      return FALSE;
    }
    modbus_flush(mb);
    return TRUE;
}
static gboolean
read_info(AppContext *app, GError **err)
{
  static const char *bps[16] = {"?", "?", "?", "1200", "2400", "4800", "9600",
				"19200", "38400", "57600", "115200"};
  static const char *ps[4] = {"N,1","N,2", "E,1", "O,1"};
  int r;
  uint16_t fw[2];
  uint16_t mod_name[2];
  uint16_t addr;
  uint16_t ser_conf;
  uint8_t wd_enabled;
  uint16_t wd_timeout;
  struct timespec wait =
    {0, 300000000};
  
  r = modbus_read_input_registers(app->mb,
				  MB_ADDR_FW_LOW, 2,
				  fw);
  if (r <= 0) {
    g_set_error(err, DGW_ERROR, DGW_ERROR_READ, "Failed to read firmware version");
    return FALSE;
  }
  printf("Firmware version: 0x%04x%04x\n", fw[1], fw[0]);

  nanosleep(&wait,NULL);
  modbus_flush(app->mb);
  r = modbus_read_input_registers(app->mb,
				  MB_ADDR_MODNAME_LOW, 2,
				  mod_name);
  if (r <= 0) {
    g_set_error(err, DGW_ERROR, DGW_ERROR_READ, "Failed to read module name");
    return FALSE;
  }
  printf("Module name: 0x%04x%04x\n", mod_name[1], mod_name[0]);
  nanosleep(&wait,NULL);
  modbus_flush(app->mb);

  r = modbus_read_registers(app->mb,
				  MB_ADDR_BUS_ADDR, 1,
				  &addr);
  if (r <= 0) {
    g_set_error(err, DGW_ERROR, DGW_ERROR_READ, "Failed to read module address");
    return FALSE;
  }
  printf("Module address: %d\n", addr);
  nanosleep(&wait,NULL);
  modbus_flush(app->mb);
  
  r = modbus_read_registers(app->mb,
				  MB_ADDR_SER_CONF, 1,
				  &ser_conf);
  if (r <= 0) {
    g_set_error(err, DGW_ERROR, DGW_ERROR_READ, "Failed to read serial port configuration");
    return FALSE;
  }
  printf("Serial port: %s,%s\n", bps[ser_conf & 0x1f], ps[(ser_conf>>6) & 0x03]);
  modbus_flush(app->mb);
  
  r = modbus_read_bits(app->mb,
		       MB_ADDR_WD_ENABLED, 1,
				  &wd_enabled);
  if (r <= 0) {
    g_set_error(err, DGW_ERROR, DGW_ERROR_READ, "Failed to read watchdog status");
    return FALSE;
  }
  printf("Watchdog %s\n",wd_enabled ? "enabled" : "disabled");
  modbus_flush(app->mb);
  
  r = modbus_read_registers(app->mb,
			    MB_ADDR_WD_TIMEOUT, 1,
			    &wd_timeout);
  if (r <= 0) {
    g_set_error(err, DGW_ERROR, DGW_ERROR_READ, "Failed to read watchdog timeout");
    return FALSE;
  }
  printf("Watchdog timeout: %.1f\n", wd_timeout/ 10.0);
  modbus_flush(app->mb);
  
  return TRUE;
}

static gboolean
set_parameters(AppContext *app, GError **err)
{
  if (app->set_serial) {
    if (!write_serial_settings(app->mb, app->set_serial, err)) {
      return FALSE;
    }
  }
  if (app->watchdog_enable || app->watchdog_disable) {
      if (!write_wd_enable(app->mb, app->watchdog_enable, err)) {
	return FALSE;
      }
  }
  if (app->watchdog_timeout > 0) {
      if (!write_watchdog_timeout(app->mb, app->watchdog_timeout, err)) {
	return FALSE;
      }
  }
  if (app->set_addr > 0) {
      if (!write_mb_addr(app->mb, app->set_addr, err)) {
	return FALSE;
      }
  }
  return TRUE;
}

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
  if (!set_parameters(&app, &err)) {
    g_printerr("Setting parameters: %s\n", err->message);
    g_clear_error(&err);
    app_cleanup(&app);
    return EXIT_FAILURE;
  }

  if (app.set_addr <= 0) {
    if (!read_info(&app, &err)) {
      g_printerr("Reading info: %s\n", err->message);
      g_clear_error(&err);
      app_cleanup(&app);
      return EXIT_FAILURE;
    }
  }

  app_cleanup(&app);
  return EXIT_SUCCESS;
}
