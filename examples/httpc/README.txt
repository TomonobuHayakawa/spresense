examples/httpc
^^^^^^^^^^^^^^
  This sample is an example of HTTP client( support both http and https).


  Configuration Pre-requisites:

    CONFIG_NETUTILS_HTTPC         - HTTP client library
    CONFIG_NETUTILS_MBEDTLS       - mbedTLS library ( If not using TLS, not needed.)

  Example Configuration:
    ( By executing "./configure.sh corvo/lte", the following settings is applied automatically. )

    CONFIG_EXAMPLES_HTTPC                 - Enable HTTPC example
    CONFIG_EXAMPLES_HTTPC_PROGNAME        - Program name.
    CONFIG_EXAMPLES_HTTPC_PRIORITY        - Example priority. Default: 100
    CONFIG_EXAMPLES_HTTPC_STACKSIZE       - Example stack size. Default: 2048

  Operation:
    HTTP client operates by command input.

  Command:

    httpc init
      Initial processing of HTTPC is done(include LTE connection establishment).

    httpc wget [protocol]  [host] [port] [path]
      Send HTTP GET to [protocol]://[host]:[port]/[path].
      ( For example, if you execute "httpc wget http www.google.com 80 /", you can get "http://www.google.com:80/".)
      
      - protocol
          http or https

      - host
          arbitrary character string ( ex. www.google.com )
          
      - port
          port (ex. 80)
      
      - path
          arbitrary character string ( ex. / )

    httpc scert [filename1] [filename2] ...
      Change rootCA certificate.
      - filename1, filename2, ...
          root CA certificate file path( should be full path)
    
    httpc enable [type]
      enable DNS cache or TLS cache
      
      - type
          dnscache      : enable DNS cache
          sessioncache  : enable TLS cache
      
    httpc disable [type]
      disable DNS cache or TLS cache
      
      - type
          dnscache      : disable DNS cache
          sessioncache  : disable TLS cache

    httpc end
      end application.

