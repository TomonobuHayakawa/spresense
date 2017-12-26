examples/streaming
^^^^^^^^^^^^^^
  This sample is an example of audio streaming application.


  Configuration Pre-requisites:
    ( By executing "./configure.sh corvo/streaming", the following settings is applied automatically. )

    <required for LTE function>
      CONFIG_NETUTILS_HTTPC
      CONFIG_LTE
      
    <required for audio function>
      CONFIG_IDLETHREAD_STACKSIZE=4096
      CONFIG_HAVE_CXX
      CONFIG_HAVE_CXXINITIALIZE
      CONFIG_LIB_ASMP
      CONFIG_SDK_AUDIO
      CONFIG_AUDIOUTILS_MANAGER
      CONFIG_AUDIOUTILS_PLAYER
      CONFIG_AUDIOUTILS_DSP_MOUNTPT = /mnt/spif
      CONFIG_MEMUTILS
      CONFIG_MEMUTILS_MEMORY_MANAGER
      CONFIG_MEMUTILS_MESSAGE
      CONFIG_MEMUTILS_SIMPLE_FIFO

    <required for combination of lte and audio>
      CONFIG_ASMP_MEMSIZE = 0xa0000

      (Default ASMP size setting is too large for combination of lte and audio.

  Example Configuration:

    CONFIG_EXAMPLES_STREAMING           - Enable streaming example
    CONFIG_EXAMPLES_STREAMING_PROGNAME  - Program name.
    CONFIG_EXAMPLES_STREAMING_PRIORITY  - Example priority. Default: 100
    CONFIG_EXAMPLES_STREAMING_STACKSIZE - Example stack size. Default: 3072

  Operation:
    streaming operates by command input.
    

  Command:

    streaming init
      Initial processing of streaming is done.
      (Include LTE connection establishment, audio power on, and reading streaming play list file "/mnt/spif/streaming_playlist.txt".

    streaming start
      Start streaming.
      
    streaming stop
      Stop streaming.

  Note:
    Please refer the streaming_play_list_format/streaming_playlist.txt about the format of streaming play iist.
