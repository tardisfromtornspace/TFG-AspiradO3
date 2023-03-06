idf_component_register(SRCS "minmea.c" "nmea_parser.c" "main.c"
                    INCLUDE_DIRS "."
                    EMBED_TXTFILES "certs/http2_telegram_root_cert.pem"
                                   "certs/cacert.pem"
                                   "certs/prvtkey.pem")
