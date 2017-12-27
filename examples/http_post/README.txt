examples/http_post
^^^^^^^^^^^^^^^^^^
  This sample is an example of HTTP client using POST method( support both http and https).


  Configuration Pre-requisites:

    CONFIG_NETUTILS_HTTPC         - HTTP client library
    CONFIG_NETUTILS_MBEDTLS       - mbedTLS library ( If not using TLS, not needed.)

  Example Configuration:
    ( By executing "./configure.sh corvo/lte", the following settings is applied automatically. )

    CONFIG_EXAMPLES_HTTP_POST                 - Enable HTTP POST example
    CONFIG_EXAMPLES_HTTP_POST_PROGNAME        - Program name.
    CONFIG_EXAMPLES_HTTP_POST_PRIORITY        - Example priority. Default: 100
    CONFIG_EXAMPLES_HTTP_POST_STACKSIZE       - Example stack size. Default: 2048

  Operation:
    HTTP client operates by command input.

  Command:

    http_post [-s] <host_address> <port_number> <path> [-f <file_name>] [-d <data>]
      Send HTTP POST to http(s)://<host_address>:<port_number><path>.
      ( For example, if you execute "http_post -s www.google.com 443 /", you can post "https://www.google.com:443/".)

      -s
          optional parameter.
          use https.

      host_address
          arbitrary character string. ( ex. www.google.com )

      port_number
          port number. (ex. 80)
      
      path
          arbitrary character string. ( ex. / )

      -f <file_name>
          optional parameter. 
          post data using specified file.

      -d <data>
          optional parameter. 
          post data direct from command line.

      # must specify either -f or -d option.
