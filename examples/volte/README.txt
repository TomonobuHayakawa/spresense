examples/volte
^^^^^^^^^^^^^^
  This sample is an example of VoLTE application.

  Configuration Pre-requisites:
    (By executing "./configure.sh corvo/volte", the following settings is applied automatically.)

    <required for LTE function>
      CONFIG_NET_LWIP
      CONFIG_LTE
      
    <required for audio function>
      CONFIG_IDLETHREAD_STACKSIZE=4096
      CONFIG_SDK_AUDIO
      CONFIG_AUDIOUTILS_MFE
      CONFIG_AUDIOUTILS_MPP
      CONFIG_DSP_MOUNTPT = /mnt/spif
      CONFIG_LIBM
      CONFIG_HAVE_CXX
      CONFIG_HAVE_CXXINITIALIZE
      CONFIG_LIB_ASMP
      CONFIG_AUDIOUTILS_MANAGER
      CONFIG_MEMUTILS_MEMORY_MANAGER
      CONFIG_MEMUTILS_MESSAGE
      CONFIG_MEMUTILS_SIMPLE_FIFO

    <required for combination of lte and audio>
      CONFIG_CXD56_RAM1_SIZE = 0xe0000
      CONFIG_CXD56_RAM2_SIZE = 0xa0000

      (Default RAM1 size setting is too small for combination of lte and audio.

  Example Configuration:

    CONFIG_EXAMPLES_VOLTE           - Enable VoLTE example
    CONFIG_EXAMPLES_VOLTE_PROGNAME  - Program name.
    CONFIG_EXAMPLES_VOLTE_PRIORITY  - Example priority. Default: 100
    CONFIG_EXAMPLES_VOLTE_STACKSIZE - Example stack size. Default: 2048
    CONFIG_CXD56_VOLTE

  Operation:
    VoLTE operates by command input.
    

  Command:

    volte init
      Initial processing of VoLTE is done.
       - Need to create 6tasks
           LTE control task      ( task main function is made in application.)
           Audio manager task    ( task main function name is fixed(= AS_AudioManagerEntry)
           Audio Player task     ( task main function name is fixed(= AS_PlayerObjEntry)
           Audio OutputMix task  ( task main function name is fixed(= AS_OutputMixObjEntry)
           Audio soundeffect task( task main function name is fixed(= AS_SoundEffectObjEntry)
           Audio control task    ( task main function is made in application.)

    volte system [parameter3] [parameter4]
      Same as lte example except that the first parameter is not "lte" but "volte")

    volte utility [parameter3] [parameter4]
      Same as lte example except that the first parameter is not "lte" but "volte")

    volte sms [parameter3] [parameter4]
      Same as lte example except that the first parameter is not "lte" but "volte")

    volte net [parameter3] [parameter4]
      Same as lte example except that the first parameter is not "lte" but "volte")

    volte volte call [URI_type] [number]
      Start VoLTE call.
      (ex.  "volte volte call sip 08011112222" )
      - URI_type
          sip : SIP URI
          tel : tel URI

      - number
          peer telephone number.
          
    volte volte answer
      Answer to VoLTE incoming call.

    volte volte reject
      Reject VoLTE incoming call.

    volte volte hold
      Hold VoLTE active call.

    volte volte resume
      Resume VoLTE held call.

    volte volte end
      End volte call.
       - Attention that volte application end, if you  execute "volte end".

    volte volte multicall [current_control] [another_control]
      Control multicall.
       - Use when there are one active call and another call(held call or incoming waiting call).
         ( When you are speaking to A-san, occur VoLTE incoming call from B-san.)

       - current_control / another_control
         current_control is control for current call(active call).
         another_control is control for held or incoming waiting call.

          hold / resume / answer / reject / disconnect / continue

         (ex.  "volte volte multicall hold answer" means "Hold current call and answer to incoming waiting call)
         
  How to make this example the VoLTE standby state:
    1) Put MP3DEC,MFESRC and ring_tone.mp3 on /mnt/spif/
    2) Execute "volte init"
    3) Execute "volte system connect"
    
