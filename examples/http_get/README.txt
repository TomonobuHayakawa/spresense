examples/http_get
^^^^^^^^^^^^^^^^^
  This sample is an example of HTTP client using GET method( support both http and https).


  Configuration Pre-requisites:

    CONFIG_NETUTILS_HTTPC         - HTTP client library
    CONFIG_NETUTILS_MBEDTLS       - mbedTLS library ( If not using TLS, not needed.)

  Example Configuration:
    ( By executing "./configure.sh corvo/lte", the following settings is applied automatically. )

    CONFIG_EXAMPLES_HTTP_GET                 - Enable HTTP GET example
    CONFIG_EXAMPLES_HTTP_GET_PROGNAME        - Program name.
    CONFIG_EXAMPLES_HTTP_GET_PRIORITY        - Example priority. Default: 100
    CONFIG_EXAMPLES_HTTP_GET_STACKSIZE       - Example stack size. Default: 2048

  Operation:
    HTTP client operates by command input.

  Command:

    http_get [-s] <host_address> <port_number> <path>
      Send HTTP GET to http(s)://<host_address>:<port_number><path>.
      ( For example, if you execute "http_get -s www.google.com 443 /", you can get "https://www.google.com:443/".)

      -s
          optional parameter.
          use https.

      host_address
          arbitrary character string. ( ex. www.google.com )

      port_number
          port number. (ex. 80)
      
      path
          arbitrary character string. ( ex. / )

