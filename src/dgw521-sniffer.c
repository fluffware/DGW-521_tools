#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdint.h>
#include <glib.h>
#include <glib-unix.h>
#include <modbus-rtu.h>

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
  gboolean debug;
  gboolean decode;
  
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
  app->debug = 0;
  app->decode = FALSE;
  app->mb = NULL;
  app->mb_thread_running = FALSE;
  g_mutex_init(&app->mb_mutex);
  g_cond_init(&app->mb_cond);
}

static void
stop_mb_thread(AppContext *app);

static void
app_cleanup(AppContext* app)
{
  stop_mb_thread(app); 
  if (app->mb) {
    modbus_close(app->mb);
    modbus_free(app->mb);
    app->mb = NULL;
  }
}

#define MB_ADDR_SEQUENCE 322
#define MB_ADDR_RECORDS 1024
/* Don't read the full buffer since the oldest records risk being overwritten.*/
#define MAX_RECORDS 24

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
    g_printerr("Failed to read first sequece number: %s", 
	       modbus_strerror(errno));
  }
  modbus_flush(app->mb);
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
	  g_printerr("Failed to read records: %s\n", modbus_strerror(errno));
	}
	
	  
	last_seq = seq;
      }
    } else {
      g_printerr("Failed to read sequece number\n");
    }
    modbus_flush(app->mb);
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

static gboolean
init_modbus(AppContext *app)
{
  app->mb = modbus_new_rtu(app->device, app->speed, 'N', 8, 1);
  if (!app->mb) {
    g_printerr("Failed to create Modbus context\n");
    return FALSE;
  }
  modbus_set_debug(app->mb, app->debug);
  modbus_set_slave(app->mb,1);
  if (modbus_connect(app->mb)) {
    g_printerr("Failed to connect: %s\n",modbus_strerror(errno));
    return FALSE;
  }
  
  app->mb_thread = g_thread_new("Modbus", modbus_poll, app);
  g_mutex_lock(&app->mb_mutex);
  // Wait until the thread has signaled it's running
  while(!app->mb_thread_running) {
    g_cond_wait (&app->mb_cond, &app->mb_mutex);
  }
  g_mutex_unlock(&app->mb_mutex);
  
  return TRUE;
}

static gboolean
sigint_handler(gpointer user_data)
{
  g_main_loop_quit(user_data);
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
  {"decode", 0, 0, G_OPTION_ARG_NONE,
   &app.decode, "Decode packets", NULL},
  {"debug", 0, 0, G_OPTION_ARG_NONE, &app.debug,
   "Turn on Modbus debugging", NULL},
  {NULL}
};

int
main(int argc, char **argv)
{
  GMainLoop *loop;
  GError *err = NULL;
  GOptionContext *opt_ctxt;
  app_init(&app);
  opt_ctxt = g_option_context_new (" - log DALI traffic");
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
  
  loop = g_main_loop_new(NULL, FALSE);
  g_unix_signal_add(SIGINT, sigint_handler, loop);
  g_unix_signal_add(SIGTERM, sigint_handler, loop);
  g_debug("Starting");
  g_main_loop_run(loop);
  g_main_loop_unref(loop);
  g_message("Exiting");     
  app_cleanup(&app);
  return EXIT_SUCCESS;
}
