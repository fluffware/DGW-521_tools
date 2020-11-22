AM_CPPFLAGS = @GLIB_CFLAGS@ @LIBMODBUS_CFLAGS@


noinst_PROGRAMS =  
bin_PROGRAMS = dgw521_sniffer dgw521_info dgw521_send

dgw521_sniffer_SOURCES = dgw521-sniffer.c
dgw521_sniffer_LDADD= @GLIB_LIBS@ @LIBMODBUS_LIBS@

dgw521_info_SOURCES = dgw521_info.c
dgw521_info_LDADD= @GLIB_LIBS@ @LIBMODBUS_LIBS@

dgw521_send_SOURCES = dgw521_send.c
dgw521_send_LDADD= @GLIB_LIBS@ @LIBMODBUS_LIBS@

